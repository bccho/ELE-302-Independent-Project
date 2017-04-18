/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionExit;
    QAction *actionSave_Image;
    QAction *actionPlay_Pause;
    QAction *actionHelp;
    QAction *actionAbout;
    QAction *actionConfigure;
    QAction *actionProgram;
    QAction *actionRaw_video;
    QAction *actionCooked_video;
    QAction *actionDefault_program;
    QAction *actionSave_Pixy_parameters;
    QAction *actionLoad_Pixy_parameters;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QGridLayout *imageLayout;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuHelp;
    QMenu *menuAction;
    QStatusBar *statusBar;
    QToolBar *toolBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(677, 626);
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionSave_Image = new QAction(MainWindow);
        actionSave_Image->setObjectName(QString::fromUtf8("actionSave_Image"));
        actionSave_Image->setEnabled(true);
        actionPlay_Pause = new QAction(MainWindow);
        actionPlay_Pause->setObjectName(QString::fromUtf8("actionPlay_Pause"));
        actionHelp = new QAction(MainWindow);
        actionHelp->setObjectName(QString::fromUtf8("actionHelp"));
        actionHelp->setEnabled(true);
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionConfigure = new QAction(MainWindow);
        actionConfigure->setObjectName(QString::fromUtf8("actionConfigure"));
        actionProgram = new QAction(MainWindow);
        actionProgram->setObjectName(QString::fromUtf8("actionProgram"));
        actionRaw_video = new QAction(MainWindow);
        actionRaw_video->setObjectName(QString::fromUtf8("actionRaw_video"));
        actionCooked_video = new QAction(MainWindow);
        actionCooked_video->setObjectName(QString::fromUtf8("actionCooked_video"));
        actionDefault_program = new QAction(MainWindow);
        actionDefault_program->setObjectName(QString::fromUtf8("actionDefault_program"));
        actionSave_Pixy_parameters = new QAction(MainWindow);
        actionSave_Pixy_parameters->setObjectName(QString::fromUtf8("actionSave_Pixy_parameters"));
        actionLoad_Pixy_parameters = new QAction(MainWindow);
        actionLoad_Pixy_parameters->setObjectName(QString::fromUtf8("actionLoad_Pixy_parameters"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        imageLayout = new QGridLayout();
        imageLayout->setSpacing(6);
        imageLayout->setObjectName(QString::fromUtf8("imageLayout"));

        gridLayout->addLayout(imageLayout, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 677, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuHelp = new QMenu(menuBar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        menuAction = new QMenu(menuBar);
        menuAction->setObjectName(QString::fromUtf8("menuAction"));
        MainWindow->setMenuBar(menuBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuAction->menuAction());
        menuBar->addAction(menuHelp->menuAction());
        menuFile->addAction(actionConfigure);
        menuFile->addAction(actionSave_Pixy_parameters);
        menuFile->addAction(actionLoad_Pixy_parameters);
        menuFile->addAction(actionSave_Image);
        menuFile->addAction(actionExit);
        menuHelp->addAction(actionHelp);
        menuHelp->addAction(actionAbout);
        menuAction->addAction(actionPlay_Pause);
        menuAction->addAction(actionDefault_program);
        menuAction->addAction(actionRaw_video);
        menuAction->addAction(actionCooked_video);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", 0, QApplication::UnicodeUTF8));
        actionSave_Image->setText(QApplication::translate("MainWindow", "Save Image", 0, QApplication::UnicodeUTF8));
        actionPlay_Pause->setText(QApplication::translate("MainWindow", "Run/Stop", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionPlay_Pause->setToolTip(QApplication::translate("MainWindow", "Run/stop", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionHelp->setText(QApplication::translate("MainWindow", "Help...", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("MainWindow", "About...", 0, QApplication::UnicodeUTF8));
        actionConfigure->setText(QApplication::translate("MainWindow", "Configure...", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionConfigure->setToolTip(QApplication::translate("MainWindow", "Configure parameters", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionProgram->setText(QApplication::translate("MainWindow", "Program...", 0, QApplication::UnicodeUTF8));
        actionRaw_video->setText(QApplication::translate("MainWindow", "Raw video", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionRaw_video->setToolTip(QApplication::translate("MainWindow", "Display raw video", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionCooked_video->setText(QApplication::translate("MainWindow", "Cooked video", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionCooked_video->setToolTip(QApplication::translate("MainWindow", "Display cooked (processed) video", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionDefault_program->setText(QApplication::translate("MainWindow", "Default program", 0, QApplication::UnicodeUTF8));
        actionSave_Pixy_parameters->setText(QApplication::translate("MainWindow", "Save Pixy parameters...", 0, QApplication::UnicodeUTF8));
        actionLoad_Pixy_parameters->setText(QApplication::translate("MainWindow", "Load Pixy parameters...", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("MainWindow", "Help", 0, QApplication::UnicodeUTF8));
        menuAction->setTitle(QApplication::translate("MainWindow", "Action", 0, QApplication::UnicodeUTF8));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
