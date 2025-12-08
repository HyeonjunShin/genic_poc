from PySide6.QtWidgets import QApplication, QMainWindow, QFileDialog, QFileSystemModel, QTreeView, QHeaderView, QMessageBox
from PySide6.QtCore import  QAbstractTableModel, Qt, QFileSystemWatcher
from main_window import Ui_MainWindow
import os
import time
import re

def format_size(size_in_bytes):
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if size_in_bytes < 1024:
            return f"{size_in_bytes:.2f} {unit}"
        size_in_bytes /= 1024
    return f"{size_in_bytes:.2f} PB"

def natural_key(value):
    if isinstance(value, int) or isinstance(value, float):
        return value
    parts = re.split(r'(\d+)', str(value))
    return [int(p) if p.isdigit() else p.lower() for p in parts]


class FileModel(QAbstractTableModel):
    def __init__(self):
        super().__init__()
        self.headers = ["", "File name", "Data size", "Created", "Label", "Etc."]
        self.sortedColumn = 1
        self.sortedOrder = Qt.AscendingOrder

        # self.files = self.load_files()

    def setRootPath(self, dirPath):
        self.directory = dirPath
        self._data = self.load_files()
        self.sort(self.sortedColumn, self.sortedOrder)

    def load_files(self):
        data = []
        for f in os.listdir(self.directory):
            path = os.path.join(self.directory, f)
            if os.path.isfile(path):
                chbox = Qt.Unchecked
                size = format_size(os.path.getsize(path))
                created = time.ctime(os.path.getctime(path))
                label = ""
                etc = ""
                data.append([chbox, f, size, created, label, etc])
        return data

    def rowCount(self, parent=None):
        return len(self._data)

    def columnCount(self, parent=None):
        return len(self.headers)

    def data(self, index, role):
        if not index.isValid():
            return None
        
        row = index.row()
        col = index.column()
        value = self._data[row][col]
        if col == 0:
            if role == Qt.CheckStateRole:
                return value
            return None
        
        if role == Qt.DisplayRole or role == Qt.EditRole:
            return value

    def headerData(self, section, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return self.headers[section]

    def setData(self, index, value, role):
        if not index.isValid():
            return False
        
        row = index.row()
        col = index.column()

        if col == 0 and role == Qt.CheckStateRole:
            self._data[row][col] = value
            self.dataChanged.emit(index, index)
            return True

        if role == Qt.EditRole:
            self._data[row][col] = value
            self.dataChanged.emit(index, index)
            return True
        
        return False

    def flags(self, index):
        col = index.column()

        if col == 0:
            return Qt.ItemIsEnabled | Qt.ItemIsSelectable |  Qt.ItemIsUserCheckable

        if col == 4:
            return Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsEditable

        return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    def sort(self, column: int, order: Qt.SortOrder):
        if column == 0:
            return
        self.sortedColumn = column
        self.sortedOrder = order

        self.layoutAboutToBeChanged.emit()
        
        try:
            self._data.sort(
                key=lambda row: natural_key(row[column]),
                reverse=(order == Qt.DescendingOrder)
            )
        except Exception as e:
            print("Sort error:", e)

        self.layoutChanged.emit()



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.watcher = QFileSystemWatcher()
        self.watcher.directoryChanged.connect(self.on_directory_changed)
        self.watcher.fileChanged.connect(self.on_directory_changed)

        self.ui.tableView.setWordWrap(False)
        self.ui.tableView.clicked.connect(self.on_row_clicked)
        header = self.ui.tableView.horizontalHeader()
        # header.setSectionResizeMode(QHeaderView.Stretch)
        header.setSectionResizeMode(QHeaderView.ResizeToContents)


        self.ui.selectDirectoryBtn.clicked.connect(self.on_testButton_click)
        self.ui.deleteButton.clicked.connect(self.on_deleteButton_click)
        # For monitoring file system changes.
        # self.model = QFileSystemModel()
        self.model = FileModel()
        # self.tree = QTreeView()
        self.ui.tableView.setSortingEnabled(True)

    def on_directory_changed(self, path):
        self.ui.terminalEdit.append(f"[INFO] Directory changed: {path}")
        self.setTableView(path)
        
        self.model.layoutChanged.emit()
        # self.model.setRootPath(path)
        # self.ui.tableView.setModel(self.model)
        # self.ui.treeView.setModel(self.model)
        # self.ui.treeView.setRootIndex(self.model.index(path))

    def setTableView(self, path):
        self.model.setRootPath(path)
        self.ui.tableView.setModel(self.model)


    def on_testButton_click(self):
        self.dirPath = QFileDialog.getExistingDirectory(self, "폴더 선택")
        if self.dirPath:
            self.ui.terminalEdit.append(f"[INFO] Selected directory: {self.dirPath}")
            self.watcher.addPath(self.dirPath)
            self.ui.pathEdit.setText(self.dirPath)
            self.setTableView(self.dirPath)

    def on_deleteButton_click(self):
        row = self.ui.tableView.currentIndex()
        if not row.isValid():
            QMessageBox.warning(self, "Warning", "Have to select the file to remove.")
            return
        
        file_name = self.model.data(self.model.index(row.row(), 0), Qt.DisplayRole)
        reply = QMessageBox.question(
            self, "Remove the file", f"Is remove the '{file_name}'file?",
            QMessageBox.Yes | QMessageBox.No
        )
        if reply == QMessageBox.No:
            return

        if row.isValid():
            file_path = os.path.join(self.dirPath, file_name)
            try:
                os.remove(file_path)
                self.ui.terminalEdit.append(f"[INFO] Deleted file: {file_path}")
                self.on_directory_changed(self.dirPath)
            except Exception as e:
                self.ui.terminalEdit.append(f"[ERROR] Could not delete file: {file_path}. Reason: {str(e)}")
        else:
            self.ui.terminalEdit.append("[WARNING] No file selected for deletion.")

    def on_row_clicked(self, index):
        row = index.row()
        check_index = self.model.index(row, 0)   # 0번 컬럼(checkbox)
        current = self.model.data(check_index, Qt.CheckStateRole)

        new_state = Qt.Checked if current == Qt.Unchecked else Qt.Unchecked
        self.model.setData(check_index, new_state, Qt.CheckStateRole)


app = QApplication([])

window = MainWindow()
window.show()

app.exec()
