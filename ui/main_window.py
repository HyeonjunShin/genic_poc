# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main_windowuvIvSp.ui'
##
## Created by: Qt User Interface Compiler version 6.8.0
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QAbstractItemView, QApplication, QHBoxLayout, QHeaderView,
    QLayout, QLineEdit, QMainWindow, QMenuBar,
    QPushButton, QSizePolicy, QSpacerItem, QStatusBar,
    QTableView, QTextEdit, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(971, 839)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setSizeConstraint(QLayout.SizeConstraint.SetMaximumSize)
        self.pathEdit = QLineEdit(self.centralwidget)
        self.pathEdit.setObjectName(u"pathEdit")
        self.pathEdit.setEnabled(True)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pathEdit.sizePolicy().hasHeightForWidth())
        self.pathEdit.setSizePolicy(sizePolicy)
        self.pathEdit.setMinimumSize(QSize(0, 20))
        self.pathEdit.setReadOnly(True)

        self.verticalLayout.addWidget(self.pathEdit)

        self.terminalEdit = QTextEdit(self.centralwidget)
        self.terminalEdit.setObjectName(u"terminalEdit")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.terminalEdit.sizePolicy().hasHeightForWidth())
        self.terminalEdit.setSizePolicy(sizePolicy1)
        self.terminalEdit.setReadOnly(True)

        self.verticalLayout.addWidget(self.terminalEdit)

        self.tableView = QTableView(self.centralwidget)
        self.tableView.setObjectName(u"tableView")
        sizePolicy1.setHeightForWidth(self.tableView.sizePolicy().hasHeightForWidth())
        self.tableView.setSizePolicy(sizePolicy1)
        self.tableView.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.tableView.setWordWrap(False)
        self.tableView.horizontalHeader().setStretchLastSection(True)

        self.verticalLayout.addWidget(self.tableView)


        self.horizontalLayout.addLayout(self.verticalLayout)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.selectDirectoryBtn = QPushButton(self.centralwidget)
        self.selectDirectoryBtn.setObjectName(u"selectDirectoryBtn")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.selectDirectoryBtn.sizePolicy().hasHeightForWidth())
        self.selectDirectoryBtn.setSizePolicy(sizePolicy2)
        self.selectDirectoryBtn.setMinimumSize(QSize(200, 50))
        self.selectDirectoryBtn.setBaseSize(QSize(0, 0))
        font = QFont()
        font.setPointSize(11)
        font.setBold(False)
        self.selectDirectoryBtn.setFont(font)

        self.verticalLayout_2.addWidget(self.selectDirectoryBtn)

        self.deleteButton = QPushButton(self.centralwidget)
        self.deleteButton.setObjectName(u"deleteButton")
        sizePolicy2.setHeightForWidth(self.deleteButton.sizePolicy().hasHeightForWidth())
        self.deleteButton.setSizePolicy(sizePolicy2)
        self.deleteButton.setMinimumSize(QSize(200, 50))
        self.deleteButton.setBaseSize(QSize(0, 0))
        self.deleteButton.setFont(font)

        self.verticalLayout_2.addWidget(self.deleteButton)

        self.makeMetadatabutton = QPushButton(self.centralwidget)
        self.makeMetadatabutton.setObjectName(u"makeMetadatabutton")
        sizePolicy2.setHeightForWidth(self.makeMetadatabutton.sizePolicy().hasHeightForWidth())
        self.makeMetadatabutton.setSizePolicy(sizePolicy2)
        self.makeMetadatabutton.setMinimumSize(QSize(200, 50))
        self.makeMetadatabutton.setFont(font)

        self.verticalLayout_2.addWidget(self.makeMetadatabutton)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.successButton = QPushButton(self.centralwidget)
        self.successButton.setObjectName(u"successButton")
        self.successButton.setMinimumSize(QSize(0, 50))
        self.successButton.setFont(font)

        self.horizontalLayout_2.addWidget(self.successButton)

        self.failButton = QPushButton(self.centralwidget)
        self.failButton.setObjectName(u"failButton")
        self.failButton.setMinimumSize(QSize(0, 50))
        self.failButton.setFont(font)

        self.horizontalLayout_2.addWidget(self.failButton)


        self.verticalLayout_2.addLayout(self.horizontalLayout_2)

        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setObjectName(u"pushButton")
        sizePolicy2.setHeightForWidth(self.pushButton.sizePolicy().hasHeightForWidth())
        self.pushButton.setSizePolicy(sizePolicy2)
        self.pushButton.setMinimumSize(QSize(200, 50))
        self.pushButton.setBaseSize(QSize(0, 0))
        self.pushButton.setFont(font)

        self.verticalLayout_2.addWidget(self.pushButton)

        self.verticalSpacer = QSpacerItem(100, 600, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer)


        self.horizontalLayout.addLayout(self.verticalLayout_2)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 971, 19))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.selectDirectoryBtn.setText(QCoreApplication.translate("MainWindow", u"Select directory", None))
        self.deleteButton.setText(QCoreApplication.translate("MainWindow", u"Delete", None))
        self.makeMetadatabutton.setText(QCoreApplication.translate("MainWindow", u"Make metadata", None))
        self.successButton.setText(QCoreApplication.translate("MainWindow", u"Success", None))
        self.failButton.setText(QCoreApplication.translate("MainWindow", u"Fail", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"PushButton", None))
    # retranslateUi

