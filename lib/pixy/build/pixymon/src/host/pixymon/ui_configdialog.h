/********************************************************************************
** Form generated from reading UI file 'configdialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONFIGDIALOG_H
#define UI_CONFIGDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QTabWidget>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ConfigDialog
{
public:
    QGridLayout *gridLayout_2;
    QDialogButtonBox *buttonBox;
    QGridLayout *gridLayout;
    QTabWidget *tabWidget;
    QWidget *pixyTab;
    QGridLayout *gridLayout_5;
    QGridLayout *pixyLayout;
    QWidget *pixymonTab;
    QGridLayout *gridLayout_4;
    QGridLayout *pixymonLayout;

    void setupUi(QDialog *ConfigDialog)
    {
        if (ConfigDialog->objectName().isEmpty())
            ConfigDialog->setObjectName(QString::fromUtf8("ConfigDialog"));
        ConfigDialog->resize(589, 305);
        gridLayout_2 = new QGridLayout(ConfigDialog);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        buttonBox = new QDialogButtonBox(ConfigDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Apply|QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        gridLayout_2->addWidget(buttonBox, 1, 0, 1, 1);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        tabWidget = new QTabWidget(ConfigDialog);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        pixyTab = new QWidget();
        pixyTab->setObjectName(QString::fromUtf8("pixyTab"));
        gridLayout_5 = new QGridLayout(pixyTab);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        pixyLayout = new QGridLayout();
        pixyLayout->setObjectName(QString::fromUtf8("pixyLayout"));

        gridLayout_5->addLayout(pixyLayout, 0, 0, 1, 1);

        tabWidget->addTab(pixyTab, QString());
        pixymonTab = new QWidget();
        pixymonTab->setObjectName(QString::fromUtf8("pixymonTab"));
        gridLayout_4 = new QGridLayout(pixymonTab);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        pixymonLayout = new QGridLayout();
        pixymonLayout->setObjectName(QString::fromUtf8("pixymonLayout"));

        gridLayout_4->addLayout(pixymonLayout, 0, 0, 1, 1);

        tabWidget->addTab(pixymonTab, QString());

        gridLayout->addWidget(tabWidget, 0, 0, 1, 1);


        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 1);


        retranslateUi(ConfigDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), ConfigDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), ConfigDialog, SLOT(reject()));

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(ConfigDialog);
    } // setupUi

    void retranslateUi(QDialog *ConfigDialog)
    {
        ConfigDialog->setWindowTitle(QApplication::translate("ConfigDialog", "Configure", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(pixyTab), QApplication::translate("ConfigDialog", "Pixy Parameters (saved on Pixy)", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(pixymonTab), QApplication::translate("ConfigDialog", "PixyMon Parameters (saved on computer)", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class ConfigDialog: public Ui_ConfigDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONFIGDIALOG_H
