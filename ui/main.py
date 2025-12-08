from PySide6.QtWidgets import QApplication, QMainWindow, QFileDialog, QFileSystemModel, QTreeView, QHeaderView, QMessageBox
from PySide6.QtCore import  QAbstractTableModel, Qt, QFileSystemWatcher, QModelIndex
from main_window import Ui_MainWindow
import os
import time
import re
import pandas as pd


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
        self.headers = ["", "file_name", "data_size", "created", "label", "etc"]
        self.sortedColumn = 1
        self.sortedOrder = Qt.AscendingOrder
        self.selectedRows = []
        self.dirPath = None
        self.metadataPath = None

    def setRootPath(self, dirPath):
        self.dirPath = dirPath
        self.metadataPath = os.path.join(self.dirPath, "metadata.csv")
        
        self.checkfiles()
        self.checkMetadata()
        
        self.sort(self.sortedColumn, self.sortedOrder)

    def checkfiles(self):
        fileNames = [fileName for fileName in os.listdir(self.dirPath) if fileName.endswith('.csv') and fileName != "metadata.csv"]
        # build rows as list of dicts to avoid dtype surprises
        rows = []
        for fileName in fileNames:
            path = os.path.join(self.dirPath, fileName)
            if os.path.isfile(path):
                checkbox = Qt.Unchecked
                size = format_size(os.path.getsize(path))
                created = time.ctime(os.path.getctime(path))
                label = ""
                etc = ""
                rows.append({
                    "": checkbox, 
                    "file_name": fileName, 
                    "data_size": size, 
                    "created": created, 
                    "label": label, 
                    "etc": etc
                })

        # create DataFrame from rows (avoids empty-column float64 issues)
        if rows:
            self._data = pd.DataFrame(rows, columns=self.headers)
        else:
            # empty dataframe with correct columns
            self._data = pd.DataFrame(columns=self.headers)

        # Ensure checkbox column can hold Qt states safely (object dtype is safe)
        if "" in self._data.columns:
            self._data[""] = self._data[""].astype(object).fillna(Qt.Unchecked)

    def checkMetadata(self):
        if os.path.exists(self.metadataPath):
            metadata = pd.read_csv(self.metadataPath, header=None, names=["file_name", "label", "etc"], keep_default_na=False)
            metadata = metadata.fillna("")
            for idx in range(len(self._data)):
                file_name = self._data.loc[idx, "file_name"]
                meta_row = metadata[metadata["file_name"] == file_name]
                if not meta_row.empty:
                    self._data.loc[idx, "label"] = meta_row["label"].values[0]
                    self._data.loc[idx, "etc"] = meta_row["etc"].values[0]
                else:
                    metadata.loc[len(metadata)] = [
                        self._data.loc[idx, "file_name"],
                        self._data.loc[idx, "label"],
                        self._data.loc[idx, "etc"]
                    ]
                    print("Found new file, updating metadata:", file_name)
            self.saveMetadata()
        else:
            self.saveMetadata()

    def saveMetadata(self):
        metadata = self._data.loc[:, ["file_name", "label", "etc"]]
        metadata.to_csv(self.metadataPath, header=False, index=False)

    # --- model interface methods remain (rowCount, columnCount, data, headerData) ---
    def rowCount(self, parent=None):
        return len(self._data)

    def columnCount(self, parent=None):
        return len(self.headers)

    def data(self, index, role):
        if not index.isValid():
            return None
        
        row = index.row()
        col = index.column()
        # use iloc but handle if out-of-range
        try:
            value = self._data.iloc[row, col]
        except Exception:
            return None

        if col == 0:
            if role == Qt.CheckStateRole:
                return value
            return None
        
        if role in (Qt.DisplayRole, Qt.EditRole):
            return value

        return None

    def headerData(self, section, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return self.headers[section]
        
        if orientation == Qt.Vertical and role == Qt.DisplayRole:
            return str(section + 1)

    # setData: keep as you had but ensure dtype safe and save metadata on edits
    def setData(self, index, value, role=Qt.EditRole):
        if not index.isValid():
            return False
        
        row = index.row()
        col = index.column()

        if col == 0 and role == Qt.CheckStateRole:
            # ensure checkbox column is object dtype
            if "" in self._data.columns:
                self._data.at[row, ""] = value
            else:
                self._data.iat[row, 0] = value
            self.dataChanged.emit(index, index, [Qt.CheckStateRole])
            return True

        if role == Qt.EditRole:
            # write value
            self._data.iat[row, col] = value
            self.saveMetadata()
            self.dataChanged.emit(index, index, [Qt.DisplayRole, Qt.EditRole])
            return True
        
        return False

    def flags(self, index):
        if not index.isValid():
            return Qt.ItemIsEnabled

        col = index.column()
        base = Qt.ItemIsEnabled | Qt.ItemIsSelectable
        if col == 0:
            return base | Qt.ItemIsUserCheckable
        if col in (4, 5):
            return base | Qt.ItemIsEditable
        return base

    def sort(self, column: int, order: Qt.SortOrder):
        if column == 0:
            return

        self.sortedColumn = column
        self.sortedOrder = order

        self.layoutAboutToBeChanged.emit()

        try:
            col_name = self._data.columns[column]

            # natural sort 적용하기 위해 transform
            sorted_df = self._data.copy()
            sorted_df["_sort_key"] = sorted_df[col_name].apply(natural_key)

            sorted_df = sorted_df.sort_values(
                by="_sort_key",
                ascending=(order == Qt.AscendingOrder)
            ).drop(columns=["_sort_key"])

            self._data = sorted_df.reset_index(drop=True)

        except Exception as e:
            print("Sort error:", e)

        self.layoutChanged.emit()

    # --- selection helpers: keep selectedRows unique and emit dataChanged so UI updates ---
    def setSelectedRow(self, rows):
        # add rows (unique) and mark checkbox checked
        for row in rows:
            if row not in self.selectedRows:
                self.selectedRows.append(row)
            # set checkbox column
            idx = self.index(row, 0)
            self._data.at[row, ""] = Qt.Checked
            self.dataChanged.emit(idx, idx, [Qt.CheckStateRole])

    def setUnselectedRow(self, rows):
        # remove rows safely and update checkbox
        for row in rows:
            if row in self.selectedRows:
                try:
                    self.selectedRows.remove(row)
                except ValueError:
                    pass
            idx = self.index(row, 0)
            self._data.at[row, ""] = Qt.Unchecked
            self.dataChanged.emit(idx, idx, [Qt.CheckStateRole])

    # --- correct removeRows override (Qt protocol) ---
    def removeRows(self, row, count, parent=QModelIndex()):
        # row: starting row index, count: number of rows to remove
        # Validate
        if count <= 0 or row < 0 or row + count - 1 >= len(self._data):
            return False

        # remove underlying files first (if file deletion fails, abort)
        for r in range(row, row + count):
            file_name = self._data.iloc[r, 1]
            file_path = os.path.join(self.dirPath, file_name)
            try:
                if os.path.exists(file_path):
                    os.remove(file_path)
            except Exception as e:
                print(f"Could not delete file: {file_path}. Reason: {str(e)}")
                return False

        # Begin remove rows for view synchronization
        self.beginRemoveRows(parent, row, row + count - 1)
        # Remove rows from dataframe
        self._data = self._data.drop(self._data.index[row:row+count]).reset_index(drop=True)
        self.endRemoveRows()

        # clear any selectedRows that referred to removed rows and remap remaining selections
        self.selectedRows = [r for r in self.selectedRows if r < row or r >= row + count]
        # remap indices in selectedRows (shift them)
        new_selected = []
        for r in self.selectedRows:
            # rows after removed block shift up by count
            if r >= row + count:
                new_selected.append(r - count)
            else:
                new_selected.append(r)
        self.selectedRows = new_selected

        # persist metadata and inform view
        self.saveMetadata()
        return True

    # convenience: delete multiple selected rows (call from UI)
    def deleteSelectedRows(self):
        if not self.selectedRows:
            return False
        # delete in descending order to avoid index shifts during successive removeRows calls
        rows_to_delete = sorted(set(self.selectedRows), reverse=True)
        for r in rows_to_delete:
            self.removeRows(r, 1)
        self.selectedRows = []
        return True


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.watcher = QFileSystemWatcher()
        self.watcher.directoryChanged.connect(self.on_directory_changed)
        self.watcher.fileChanged.connect(self.on_directory_changed)

        self.ui.tableView.setWordWrap(False)
        self.ui.tableView.verticalHeader().setVisible(True)
        self.ui.tableView.verticalHeader().setDefaultSectionSize(25)
        self.ui.tableView.verticalHeader().setSectionResizeMode(QHeaderView.Fixed)

        # self.ui.tableView.clicked.connect(self.on_row_clicked)
        self.ui.tableView.doubleClicked.connect(self.on_row_clicked)

        header = self.ui.tableView.horizontalHeader()
        # header.setSectionResizeMode(QHeaderView.Stretch)
        header.setSectionResizeMode(QHeaderView.ResizeToContents)


        self.ui.selectDirectoryBtn.clicked.connect(self.on_testButton_click)
        self.ui.deleteButton.clicked.connect(self.on_deleteButton_click)
        # For monitoring file system changes.
        self.model = FileModel()
        self.ui.tableView.setSortingEnabled(True)

    def on_directory_changed(self, path):
        self.ui.terminalEdit.append(f"[INFO] Directory changed: {path}")
        self.setTableView(path)
        self.model.checkMetadata()
        self.model.layoutChanged.emit()

    def setTableView(self, path):
        self.model.setRootPath(path)
        self.ui.tableView.setModel(self.model)

        # disconnect previous connection (if any) to avoid duplicate connects
        try:
            self.ui.tableView.selectionModel().selectionChanged.disconnect(self.on_selection_changed)
        except Exception:
            pass
        self.ui.tableView.selectionModel().selectionChanged.connect(self.on_selection_changed)


    def on_testButton_click(self):
        self.dirPath = QFileDialog.getExistingDirectory(self, "폴더 선택")
        if self.dirPath:
            self.ui.terminalEdit.append(f"[INFO] Selected directory: {self.dirPath}")
            self.watcher.addPath(self.dirPath)
            self.ui.pathEdit.setText(self.dirPath)
            self.setTableView(self.dirPath)

    def on_deleteButton_click(self):
        nSelected = len(self.model.selectedRows)
        if nSelected == 0:
            QMessageBox.warning(self, "Warning", "Have to select the file to remove.")
            return

        reply = QMessageBox.question(
            self, "Delete confirmation", f"Remove the {nSelected} selected file(s)?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.No:
            return

        if not self.model.deleteSelectedRows():
            QMessageBox.warning(self, "Warning", "Could not remove the selected files.")
            return


    def on_row_clicked(self, index):
        col = index.column()
        if col > 3:
            return
        row = index.row()
        check_index = self.model.index(row, 0)
        current = self.model.data(check_index, Qt.CheckStateRole)

        new_state = Qt.Checked if current == Qt.Unchecked else Qt.Unchecked
        self.model.setData(check_index, new_state, Qt.CheckStateRole)

    def on_selection_changed(self, selected, deselected):
        selectedRows = [index.row() for index in selected.indexes()]
        unselectedRows = [index.row() for index in deselected.indexes()]
        
        selectedRows = list(set(selectedRows))
        unselectedRows = list(set(unselectedRows))

        self.model.setSelectedRow(selectedRows)
        self.model.setUnselectedRow(unselectedRows)


app = QApplication([])

window = MainWindow()
window.show()

app.exec()
