# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'at.ui'
#
# Created: Wed Sep 10 17:26:43 2014
#      by: PyQt5 UI code generator 5.3.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_at_auto(object):
    def setupUi(self, at_auto):
        at_auto.setObjectName("at_auto")
        at_auto.resize(400, 300)
        self.gridLayoutWidget = QtWidgets.QWidget(at_auto)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(10, 10, 371, 271))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.schedule = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.schedule.setObjectName("schedule")
        self.gridLayout.addWidget(self.schedule, 1, 0, 1, 1)
        self.command = QtWidgets.QLineEdit(self.gridLayoutWidget)
        self.command.setObjectName("command")
        self.gridLayout.addWidget(self.command, 0, 0, 1, 1)
        self.time = QtWidgets.QDateTimeEdit(self.gridLayoutWidget)
        self.time.setObjectName("time")
        self.gridLayout.addWidget(self.time, 2, 0, 1, 1)

        self.retranslateUi(at_auto)
        QtCore.QMetaObject.connectSlotsByName(at_auto)

    def retranslateUi(self, at_auto):
        _translate = QtCore.QCoreApplication.translate
        at_auto.setWindowTitle(_translate("at_auto", "Form"))
        self.schedule.setText(_translate("at_auto", "PushButton"))

