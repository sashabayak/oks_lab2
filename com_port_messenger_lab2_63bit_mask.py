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


# ============================================================
# СТРУКТУРА КАДРА
# ============================================================

class Frame:
    """
    Структура кадра (ровно 100 байт до бит-стаффинга):

    1. Флаг начала кадра (8 байт: "450502-1")
    2. Тип кадра (1 байт)
    3. Номер кадра (1 байт)
    4. Контрольная сумма (1 байт)
    5. Поле данных (до 79 байт, дополняется нулями до 79)
    6. 10 зарезервированных байтов

    Смысл служебных полей (предусмотрены, пока передаются нулевыми):
    - тип_кадра: вид содержимого (обычные данные / команда);
    - номер_кадра: порядковый номер для контроля порядка доставки;
    - контрольная_сумма: проверка целостности поля данных.
    """

    def __init__(self, data: bytes):
        if len(data) > MAX_DATA_SIZE:
            raise ValueError("Поле данных превышает допустимый размер.")

        self.flag = FRAME_FLAG
        self.data = data

        # Пока не используются — передаются нулевыми
        self.frame_type = 0
        self.frame_number = 0
        self.checksum = 0
        self.reserved = bytes(RESERVED_SIZE)

    def padded_data(self) -> bytes:
        return self.data.ljust(MAX_DATA_SIZE, b'\x00')

    def to_bytes(self) -> bytes:
        """Физический кадр ровно 100 байт (до стаффинга)."""
        body = (
            bytes([self.frame_type])
            + bytes([self.frame_number])
            + bytes([self.checksum])
            + self.padded_data()
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
    return ''.join(format(byte, '08b') for byte in data)


# Маска состоит из первых 63 бит флага. Его последний бит равен 1,
# поэтому при совпадении с маской в тело вставляется 0.
FLAG_BITS = bytes_to_bits(FRAME_FLAG)
FLAG_MASK = FLAG_BITS[:-1]
MASK_SIZE = len(FLAG_MASK)


def bits_to_bytes(bits: str) -> bytes:
    if len(bits) % 8 != 0:
        bits = bits.ljust(len(bits) + (8 - len(bits) % 8), '0')
    return bytes(int(bits[i:i + 8], 2) for i in range(0, len(bits), 8))


def bit_stuff(bits: str) -> str:
    """Вставляет 0 после каждого совпадения последних 63 битов с маской флага."""
    result = []
    tail = ''
    for bit in bits:
        result.append(bit)
        tail = (tail + bit)[-MASK_SIZE:]
        if tail == FLAG_MASK:
            result.append('0')
            tail = (tail + '0')[-MASK_SIZE:]
    return ''.join(result)


def format_field_bits(data: bytes) -> str:
    return ','.join(format(b, '08b') for b in data)


def format_stuffed_field(data: bytes, history: str = ''):
    """Показывает вставленные нули подчёркнутыми и переносит историю между полями."""
    byte_parts = []
    tail = history[-MASK_SIZE:]
    for byte in data:
        tokens = []
        for bit in format(byte, '08b'):
            tokens.append(bit)
            tail = (tail + bit)[-MASK_SIZE:]
            if tail == FLAG_MASK:
                tokens.append('<u>0</u>')
                tail = (tail + '0')[-MASK_SIZE:]
        byte_parts.append(''.join(tokens))
    return ','.join(byte_parts), tail


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
        self.tx_symbol_buffer = 0
        self.tx_buffer = bytearray()

        self.rx_buffer = bytearray()
        self.receiving_frame = False
        self.received_bits = []
        self.expect_mask_zero = False

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

        self.status_label = QLabel("Передано символов: 0")
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
            self.tx_symbol_buffer = 0
            self.rx_buffer.clear()
            self.receiving_frame = False
            self.received_bits = []
            self.expect_mask_zero = False

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
            self.tx_symbol_buffer += 1

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

            # Флаг без стаффинга; стаффится только тело кадра
            body = frame_bytes[FLAG_SIZE:]
            stuffed_body = bits_to_bytes(bit_stuff(bytes_to_bits(body)))
            self.serial.write(FRAME_FLAG + stuffed_body)

            self.tx_count += self.tx_symbol_buffer
            self.status_label.setText(f"Передано символов: {self.tx_count}")
            self.show_last_frame(frame)

            self.tx_buffer.clear()
            self.tx_symbol_buffer = 0

        except Exception as e:
            self.show_error(f"Ошибка передачи: {e}")

    # ========================================================
    # ОКНО СТАТУСА
    # ========================================================

    def show_last_frame(self, frame: Frame):
        """
        Названия полей, затем последний кадр до и после бит-стаффинга.
        Зарезервированные байты не выводятся. Формат — двоичный.
        Вставленные биты подчёркнуты.
        """
        flag_text = format_field_bits(frame.flag)

        type_before = format(frame.frame_type, '08b')
        number_before = format(frame.frame_number, '08b')
        checksum_before = format(frame.checksum, '08b')
        data_before = format_field_bits(frame.data) if frame.data else ''

        before = (
            f"{flag_text} {type_before} {number_before} {checksum_before} "
            f"{data_before}"
        )

        # Стаффинг выполняется непрерывно в порядке полей кадра.
        history = ''
        type_after, history = format_stuffed_field(bytes([frame.frame_type]), history)
        number_after, history = format_stuffed_field(bytes([frame.frame_number]), history)
        checksum_after, history = format_stuffed_field(bytes([frame.checksum]), history)
        data_after, history = format_stuffed_field(frame.data, history)

        after = (
            f"{flag_text} {type_after} {number_after} {checksum_after} "
            f"{data_after}"
        )

        # Заменяем обычные пробелы на неразрывные, чтобы браузер
        # не переносил строку после флага (и между полями вообще)
        before_html = before.replace(' ', '&nbsp;')
        after_html = after.replace(' ', '&nbsp;')

        html = (
            '<div style="white-space: pre-wrap;">'
            'Флаг Тип_кадра Номер_кадра Контрольная_сумма Данные<br>'
            f'{before_html}<br>'
            f'{after_html}'
            '</div>'
        )
        self.frame_state.setHtml(html)

    # ========================================================
    # ПРИЁМ И ДЕСТАФФИНГ
    # ========================================================

    def _reset_rx_frame(self):
        self.receiving_frame = False
        self.received_bits = []
        self.expect_mask_zero = False

    def receive_data(self, raw_data: bytes):
        self.rx_buffer.extend(raw_data)

        while True:
            if not self.receiving_frame:
                position = self.rx_buffer.find(FRAME_FLAG)
                if position == -1:
                    if len(self.rx_buffer) > FLAG_SIZE:
                        self.rx_buffer = self.rx_buffer[-(FLAG_SIZE - 1):]
                    return

                del self.rx_buffer[:position + FLAG_SIZE]
                self.receiving_frame = True
                self.received_bits = []
                self.expect_mask_zero = False

            if not self.rx_buffer:
                return

            current_byte = self.rx_buffer.pop(0)
            frame_finished = False
            frame_error = False

            for bit in format(current_byte, '08b'):
                if self.expect_mask_zero:
                    if bit != '0':
                        frame_error = True
                        break
                    self.expect_mask_zero = False
                    if len(self.received_bits) >= BODY_BITS:
                        frame_finished = True
                        break
                    continue

                self.received_bits.append(bit)

                if (len(self.received_bits) >= MASK_SIZE and
                        ''.join(self.received_bits[-MASK_SIZE:]) == FLAG_MASK):
                    self.expect_mask_zero = True

                if len(self.received_bits) >= BODY_BITS and not self.expect_mask_zero:
                    frame_finished = True
                    break

            if frame_error:
                self._reset_rx_frame()
                continue

            if not frame_finished:
                continue

            body = bits_to_bytes(''.join(self.received_bits[:BODY_BITS]))
            data_start = 3 * SERVICE_FIELD_SIZE
            data = body[data_start:data_start + MAX_DATA_SIZE].rstrip(b'\x00')

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
