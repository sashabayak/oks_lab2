import sys
import serial
import serial.tools.list_ports

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QLabel, QComboBox, QTextEdit,
    QMessageBox, QGroupBox
)
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtGui import QTextCursor


# ============================================================
# ПАРАМЕТРЫ КАДРА (Вариант 1, Группа 450502)
# ============================================================

# 45050 + последняя цифра группы (2) + "-" + вариант (1)
FRAME_FLAG = b"450502-1"
FLAG_SIZE = 8

# Вариант 1: максимальная длина кадра — 100 байт
MAX_FRAME_SIZE = 100

SERVICE_FIELD_SIZE = 1
RESERVED_SIZE = 10

# 100 - 8 - 1 - 1 - 1 - 10 = 79 байт
MAX_DATA_SIZE = (
    MAX_FRAME_SIZE
    - FLAG_SIZE
    - 3 * SERVICE_FIELD_SIZE
    - RESERVED_SIZE
)

BODY_SIZE = MAX_DATA_SIZE + 3 * SERVICE_FIELD_SIZE + RESERVED_SIZE
BODY_BITS = BODY_SIZE * 8
PAYLOAD_SIZE = BODY_SIZE - RESERVED_SIZE
# ============================================================
# СТРУКТУРА КАДРА
# ============================================================

class Frame:
    """
    Структура кадра (ровно 100 байт до бит-стаффинга):

    1. Флаг начала кадра (8 байт: "450502-1")
    2. Поле данных (до 79 байт, дополняется нулями до 79)
    3. Тип кадра (1 байт)
    4. Номер кадра (1 байт)
    5. Контрольная сумма (1 байт)
    6. 10 зарезервированных байтов

    Смысл служебных полей (сейчас передаются нулевыми):
    - тип_кадра: вид содержимого (обычные данные / команда);
    - номер_кадра: порядковый номер для контроля порядка доставки;
    - контрольная_сумма: проверка целостности поля данных.

    Зарезервированные 10 байтов сейчас не используются и всегда
    передаются нулевыми; в дальнейшем они могут быть выделены
    под дополнительные служебные параметры протокола.
    """

    def __init__(self, data: bytes):
        if len(data) > MAX_DATA_SIZE:
            raise ValueError("Поле данных превышает допустимый размер.")

        self.flag = FRAME_FLAG
        self.data = data

        self.frame_type = 0
        self.frame_number = 0
        self.checksum = 0
        self.reserved = bytes(RESERVED_SIZE)

    def padded_data(self) -> bytes:
        return self.data.ljust(MAX_DATA_SIZE, b'\x00')

    def to_bytes(self) -> bytes:
        """Физический кадр ровно 100 байт (до стаффинга)."""
        body = (
            self.padded_data()
            + bytes([self.frame_type])
            + bytes([self.frame_number])
            + bytes([self.checksum])
            + self.reserved
        )
        frame = self.flag + body
        if len(frame) != MAX_FRAME_SIZE:
            raise ValueError("Длина кадра должна быть ровно 100 байт.")
        return frame


# ============================================================
# ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
# ============================================================

def bytes_to_bits(data: bytes) -> str:
    if isinstance(data, str):
        data = data.encode('utf-8')
    return ''.join(format(int(byte), '08b') for byte in data)


def bits_to_bytes(bits: str) -> bytes:
    if len(bits) % 8 != 0:
        bits = bits.ljust(len(bits) + (8 - len(bits) % 8), '0')
    result = bytearray()
    for i in range(0, len(bits), 8):
        result.append(int(bits[i:i + 8], 2))
    return bytes(result)


def flag_bits() -> str:
    return bytes_to_bits(FRAME_FLAG)


FLAG_BITS = flag_bits()
DANGEROUS_PREFIX = FLAG_BITS[:-1]


def optimized_bit_stuff(bits: str):
    """
    Оригинальный самодостаточный бит-стаффинг.

    Если следующие 63 бита совпадают с первыми 63 битами флага,
    перед следующим исходным битом вставляется 0.

    Поэтому передача никогда не содержит полный флаг внутри тела:
    DANGEROUS_PREFIX + 0 + исходный_бит.

    Вставляется только один бит на потенциальное совпадение с флагом.
    Это существенно уменьшает число изменений по сравнению с
    классическим правилом «после каждых пяти единиц вставлять 0».
    """
    stuffed = []
    inserted_positions = []
    i = 0

    while i < len(bits):
        if bits.startswith(DANGEROUS_PREFIX, i):
            stuffed.extend(DANGEROUS_PREFIX)
            stuffed.append('0')
            inserted_positions.append(len(stuffed) - 1)
            i += len(DANGEROUS_PREFIX)
        else:
            stuffed.append(bits[i])
            i += 1

    result = ''.join(stuffed)

    if FLAG_BITS in result:
        raise ValueError("Оптимизированный бит-стаффинг не устранил флаг.")

    return result, inserted_positions


def destuff_body_bits(bits: str, target_size: int):
    """
    Обратное преобразование.
    Возвращает:
    - исходные биты тела;
    - количество использованных переданных битов;
    - None, если данных пока недостаточно;
    - ValueError при ошибочном кадре.
    """
    result = []
    i = 0

    while i < len(bits) and len(result) < target_size:
        remaining = len(bits) - i

        if remaining < len(DANGEROUS_PREFIX):
            return None

        if bits.startswith(DANGEROUS_PREFIX, i):
            if remaining < len(DANGEROUS_PREFIX) + 2:
                return None

            marker = bits[i + len(DANGEROUS_PREFIX)]
            if marker != '0':
                raise ValueError("Некорректный маркер бит-стаффинга.")

            original_bit = bits[i + len(DANGEROUS_PREFIX) + 1]
            result.extend(DANGEROUS_PREFIX)
            result.append(original_bit)
            i += len(DANGEROUS_PREFIX) + 2
        else:
            result.append(bits[i])
            i += 1

    if len(result) < target_size:
        return None

    return ''.join(result[:target_size]), i


def format_stuffed_fields(fields):
    """
    Форматирует поля после бит-стаффинга.
    Вставленные нули выделяются подчеркиванием.
    Стаффинг выполняется над общей последовательностью битов,
    поэтому границы полей не сбрасывают состояние алгоритма.
    """
    all_bits = ''.join(fields)

    bounds = []
    start = 0
    for field in fields:
        end = start + len(field)
        bounds.append((start, end))
        start = end

    chunks = [[] for _ in fields]
    i = 0

    def field_index(position):
        for index, (left, right) in enumerate(bounds):
            if left <= position < right:
                return index
        return len(bounds) - 1

    while i < len(all_bits):
        if all_bits.startswith(DANGEROUS_PREFIX, i):
            for j, bit in enumerate(DANGEROUS_PREFIX):
                chunks[field_index(i + j)].append(bit)

            marker_field = field_index(i + len(DANGEROUS_PREFIX))
            chunks[marker_field].append("<u>0</u>")
            i += len(DANGEROUS_PREFIX)
        else:
            chunks[field_index(i)].append(all_bits[i])
            i += 1

    result = []
    for tokens in chunks:
        values = []
        for pos in range(0, len(tokens), 8):
            values.append(''.join(tokens[pos:pos + 8]))
        result.append(','.join(values))

    return result, None


def format_field_bits(data: bytes) -> str:
    if isinstance(data, str):
        data = data.encode('utf-8')
    return ','.join(format(int(b), '08b') for b in data)


# ============================================================
# ПОТОК ПРИЁМА
# ============================================================

class SerialReaderThread(QThread):

    data_received = pyqtSignal(bytes)
    error_occurred = pyqtSignal(str)

    def __init__(self, serial_port):
        super().__init__()
        self.serial_port = serial_port
        self.running = True

    def run(self):
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.read(self.serial_port.in_waiting)
                        if data:
                            self.data_received.emit(data)
                    else:
                        self.msleep(40)
                except Exception as e:
                    self.error_occurred.emit(f"Ошибка чтения данных: {e}")
                    self.msleep(1000)

    def stop(self):
        self.running = False
        self.wait()


# ============================================================
# ПОЛЕ ВВОДА (как в main.py / лаб. №1)
# ============================================================

class CharTextEdit(QTextEdit):

    char_pressed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.cursorPositionChanged.connect(self._lock_cursor_to_end)

    def _lock_cursor_to_end(self):
        cursor = self.textCursor()
        if not cursor.atEnd():
            cursor.movePosition(QTextCursor.MoveOperation.End)
            self.setTextCursor(cursor)

    def mousePressEvent(self, event):
        super().mousePressEvent(event)
        self.moveCursor(QTextCursor.MoveOperation.End)

    def mouseDoubleClickEvent(self, event):
        self.moveCursor(QTextCursor.MoveOperation.End)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key.Key_Backspace, Qt.Key.Key_Delete):
            event.accept()
            return

        forbidden_keys = (
            Qt.Key.Key_Left,
            Qt.Key.Key_Up,
            Qt.Key.Key_Home,
            Qt.Key.Key_PageUp
        )
        if event.key() in forbidden_keys:
            self.moveCursor(QTextCursor.MoveOperation.End)
            event.accept()
            return

        if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
            event.accept()
            return

        if event.text():
            self.char_pressed.emit(event.text())

        super().keyPressEvent(event)
        self.moveCursor(QTextCursor.MoveOperation.End)


# ============================================================
# ОСНОВНОЕ ОКНО
# ============================================================

class ComPortApp(QMainWindow):

    def __init__(self):
        super().__init__()

        self.serial = serial.Serial()
        self.reader_thread = None
        self.tx_count = 0
        self.tx_buffer = bytearray()

        self.rx_buffer = bytearray()
        self.rx_bit_buffer = ""
        self.receiving_frame = False

        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("COM-порт Мессенджер_2")
        self.resize(560, 480)
        self.setStyleSheet("QWidget { font-size: 14px; }")

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        control_group = QGroupBox("Окно управления")
        ctrl_layout = QHBoxLayout()

        self.port_combo = QComboBox()
        self.port_combo.setEditable(False)
        self.update_ports()
        self.port_combo.currentTextChanged.connect(self.try_lock_and_open_port)

        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems([
            "110", "300", "600", "1200", "2400", "4800", "9600",
            "14400", "19200", "38400", "57600", "115200"
        ])
        self.baudrate_combo.setCurrentText("9600")
        self.baudrate_combo.currentTextChanged.connect(self.update_baudrate)

        ctrl_layout.addWidget(QLabel("COM-порт:"))
        ctrl_layout.addWidget(self.port_combo)
        ctrl_layout.addWidget(QLabel("Скорость:"))
        ctrl_layout.addWidget(self.baudrate_combo)
        control_group.setLayout(ctrl_layout)
        layout.addWidget(control_group)

        layout.addWidget(QLabel("Окно ввода (посимвольная передача):"))
        self.input_field = CharTextEdit()
        self.input_field.setMinimumHeight(100)
        self.input_field.char_pressed.connect(self.send_data)
        layout.addWidget(self.input_field)

        layout.addWidget(QLabel("Окно вывода (принятые сообщения):"))
        self.output_field = QTextEdit()
        self.output_field.setReadOnly(True)
        layout.addWidget(self.output_field)

        # Окно статуса как в main.py + дополнение для лаб.2 (поля кадра)
        status_group = QGroupBox("Окно статуса")
        status_layout = QVBoxLayout()

        self.status_label = QLabel("Передано кадров: 0")
        self.status_label.setStyleSheet("font-weight: bold;")
        status_layout.addWidget(self.status_label)

        self.frame_state = QTextEdit()
        self.frame_state.setReadOnly(True)
        status_layout.addWidget(self.frame_state)

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        self.frame_state.setPlainText(
            "Флаг Данные Тип_кадра Номер_кадра Контрольная_сумма"
        )

    # ========================================================
    # COM-ПОРТЫ
    # ========================================================

    def update_ports(self):
        self.port_combo.clear()
        self.port_combo.addItem("")
        ports = [port.device for port in serial.tools.list_ports.comports()]
        if ports:
            self.port_combo.addItems(sorted(ports))
        self.port_combo.setCurrentIndex(0)

    def get_baudrate(self):
        try:
            return int(self.baudrate_combo.currentText())
        except ValueError:
            return 9600

    def try_lock_and_open_port(self, port_name):
        port_name = port_name.strip()
        if not port_name:
            return

        try:
            if self.serial.is_open:
                if self.reader_thread:
                    self.reader_thread.stop()
                    self.reader_thread = None
                self.serial.close()

            self.serial.port = port_name
            self.serial.baudrate = self.get_baudrate()
            self.serial.parity = serial.PARITY_NONE
            self.serial.stopbits = serial.STOPBITS_ONE
            self.serial.bytesize = serial.EIGHTBITS
            self.serial.timeout = 0.1
            self.serial.open()

            self.port_combo.setEnabled(False)

            self.tx_buffer.clear()
            self.rx_buffer.clear()
            self.rx_bit_buffer = ""
            self.receiving_frame = False

            self.reader_thread = SerialReaderThread(self.serial)
            self.reader_thread.data_received.connect(self.receive_data)
            self.reader_thread.error_occurred.connect(self.show_error)
            self.reader_thread.start()

        except Exception as e:
            self.show_error(f"Не удалось открыть порт: {e}")
            self.port_combo.setCurrentIndex(0)

    def update_baudrate(self):
        if not self.serial.is_open:
            return
        try:
            self.serial.baudrate = self.get_baudrate()
        except Exception as e:
            self.show_error(f"Не удалось изменить скорость: {e}")

    # ========================================================
    # ПЕРЕДАЧА: кадр отправляется после полного заполнения поля данных
    # ========================================================

    def send_data(self, char: str):
        if char in ('\n', '\r'):
            return

        if not self.serial.is_open:
            self.show_error("Сначала выберите и откройте COM-порт.")
            return

        try:
            data = char.encode('utf-8')

            if len(data) > MAX_DATA_SIZE:
                self.show_error("Символ слишком большой для поля данных.")
                return

            if len(self.tx_buffer) + len(data) > MAX_DATA_SIZE:
               if self.tx_buffer:
                    self.send_full_frame()

            self.tx_buffer.extend(data)

            # Если буфер заполнился ровно до MAX_DATA_SIZE — отправить
            if len(self.tx_buffer) == MAX_DATA_SIZE:
                self.send_full_frame()

            
        except Exception as e:
            self.show_error(f"Ошибка передачи: {e}")

    # ========================================================
    # ОТПРАВКА ТЕКУЩЕГО БУФЕРА КАК КАДРА
    # ========================================================

    def send_full_frame(self):
        try:
            if not self.tx_buffer:
                return

            frame = Frame(bytes(self.tx_buffer))
            frame_bytes = frame.to_bytes()

            body = frame_bytes[FLAG_SIZE:]
            body_bits = bytes_to_bits(body)

            stuffed_bits, inserted_positions = optimized_bit_stuff(body_bits)
            stuffed_body = bits_to_bytes(stuffed_bits)

            self.serial.write(FRAME_FLAG + stuffed_body)

            self.tx_count += 1
            self.status_label.setText(f"Передано кадров: {self.tx_count}")
            self.show_last_frame(frame, stuffed_bits)

            self.tx_buffer.clear()

        except Exception as e:
            self.show_error(f"Ошибка передачи: {e}")

    # ========================================================
    # ОКНО СТАТУСА
    # ========================================================

    def show_last_frame(self, frame: Frame, stuffed_bits: str):
        flag_text = format_field_bits(frame.flag)

        data_before = format_field_bits(frame.data) if frame.data else ''
        type_before = format(int(frame.frame_type), '08b')
        number_before = format(int(frame.frame_number), '08b')
        checksum_before = format(int(frame.checksum), '08b')

        before = (
            f"{flag_text} {data_before} "
            f"{type_before} {number_before} {checksum_before}"
        )

        fields = [
            bytes_to_bits(frame.padded_data()),
            bytes([frame.frame_type]),
            bytes([frame.frame_number]),
            bytes([frame.checksum]),
        ]

        field_bits = [bytes_to_bits(fields[0])]
        field_bits.append(bytes_to_bits(fields[1]))
        field_bits.append(bytes_to_bits(fields[2]))
        field_bits.append(bytes_to_bits(fields[3]))

        stuffed_fields, _ = format_stuffed_fields(field_bits)

        after = (
            f"{flag_text} {stuffed_fields[0]} "
            f"{stuffed_fields[1]} {stuffed_fields[2]} {stuffed_fields[3]}"
        )

        before_html = before.replace(' ', '&nbsp;')
        after_html = after.replace(' ', '&nbsp;')

        html = (
            '<div style="white-space: pre-wrap;">'
            'Флаг Данные Тип_кадра Номер_кадра Контрольная_сумма<br>'
            f'{before_html}<br>'
            f'{after_html}'
            '</div>'
        )
        self.frame_state.setHtml(html)

    # ========================================================
    # ПРИЁМ И ОБРАТНОЕ ПРЕОБРАЗОВАНИЕ БИТ-СТАФФИНГА
    # ========================================================

    def _reset_rx_frame(self):
        self.receiving_frame = False

    def receive_data(self, raw_data: bytes):
        self.rx_bit_buffer += bytes_to_bits(raw_data)

        while True:
            if not self.receiving_frame:
                position = self.rx_bit_buffer.find(FLAG_BITS)

                if position == -1:
                    if len(self.rx_bit_buffer) > len(FLAG_BITS):
                        self.rx_bit_buffer = self.rx_bit_buffer[-(len(FLAG_BITS) - 1):]
                    return

                self.rx_bit_buffer = self.rx_bit_buffer[
                    position + len(FLAG_BITS):
                ]
                self.receiving_frame = True

            decoded = destuff_body_bits(
                self.rx_bit_buffer,
                BODY_BITS
            )

            if decoded is None:
                return

            body_bits, consumed = decoded
            self.rx_bit_buffer = self.rx_bit_buffer[consumed:]

            body = bits_to_bytes(body_bits)

            payload = body[:PAYLOAD_SIZE]
            data = payload[:MAX_DATA_SIZE].rstrip(b'\x00')

            if data:
                text = data.decode('utf-8', errors='replace')
                self.output_field.insertPlainText(text)
                self.output_field.ensureCursorVisible()

            self._reset_rx_frame()

    def show_error(self, message):
        QMessageBox.critical(self, "Ошибка", message)

    def closeEvent(self, event):
        if self.reader_thread:
            self.reader_thread.stop()
            self.reader_thread = None
        if self.serial.is_open:
            self.serial.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ComPortApp()
    window.show()
    sys.exit(app.exec())
