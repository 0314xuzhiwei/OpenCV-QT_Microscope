/********************************************************************************
** Form generated from reading UI file 'smollwidget.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SMOLLWIDGET_H
#define UI_SMOLLWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_smollwidget
{
public:
    QGridLayout *gridLayout;
    QLabel *label_2;
    QComboBox *comboBox;
    QLabel *label;
    QComboBox *PortBox;
    QPushButton *pushButton_2;
    QPushButton *OpenSerialButton;
    QTextEdit *DataReceived;
    QPushButton *pushButton;

    void setupUi(QWidget *smollwidget)
    {
        if (smollwidget->objectName().isEmpty())
            smollwidget->setObjectName(QString::fromUtf8("smollwidget"));
        smollwidget->resize(329, 213);
        QFont font;
        font.setFamily(QString::fromUtf8("\345\256\213\344\275\223"));
        font.setPointSize(12);
        smollwidget->setFont(font);
        gridLayout = new QGridLayout(smollwidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        label_2 = new QLabel(smollwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setMinimumSize(QSize(100, 0));
        label_2->setMaximumSize(QSize(100, 16777215));

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        comboBox = new QComboBox(smollwidget);
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->addItem(QString());
        comboBox->setObjectName(QString::fromUtf8("comboBox"));

        gridLayout->addWidget(comboBox, 0, 1, 1, 1);

        label = new QLabel(smollwidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 1, 0, 1, 1);

        PortBox = new QComboBox(smollwidget);
        PortBox->addItem(QString());
        PortBox->setObjectName(QString::fromUtf8("PortBox"));
        PortBox->setMinimumSize(QSize(0, 25));
        PortBox->setMaximumSize(QSize(16777215, 40));

        gridLayout->addWidget(PortBox, 1, 1, 1, 1);

        pushButton_2 = new QPushButton(smollwidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setMinimumSize(QSize(0, 40));

        gridLayout->addWidget(pushButton_2, 2, 0, 1, 1);

        OpenSerialButton = new QPushButton(smollwidget);
        OpenSerialButton->setObjectName(QString::fromUtf8("OpenSerialButton"));
        OpenSerialButton->setMinimumSize(QSize(0, 40));
        OpenSerialButton->setMaximumSize(QSize(16777215, 40));

        gridLayout->addWidget(OpenSerialButton, 2, 1, 1, 1);

        DataReceived = new QTextEdit(smollwidget);
        DataReceived->setObjectName(QString::fromUtf8("DataReceived"));
        DataReceived->setMinimumSize(QSize(0, 40));
        DataReceived->setMaximumSize(QSize(500, 40));

        gridLayout->addWidget(DataReceived, 3, 0, 1, 2);

        pushButton = new QPushButton(smollwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setMinimumSize(QSize(0, 40));

        gridLayout->addWidget(pushButton, 4, 0, 1, 2);


        retranslateUi(smollwidget);

        QMetaObject::connectSlotsByName(smollwidget);
    } // setupUi

    void retranslateUi(QWidget *smollwidget)
    {
        smollwidget->setWindowTitle(QCoreApplication::translate("smollwidget", "Form", nullptr));
        label_2->setText(QCoreApplication::translate("smollwidget", "\345\200\215\346\225\260\351\200\211\346\213\251\357\274\232", nullptr));
        comboBox->setItemText(0, QCoreApplication::translate("smollwidget", "16\345\200\215", nullptr));
        comboBox->setItemText(1, QCoreApplication::translate("smollwidget", "16.5\345\200\215", nullptr));
        comboBox->setItemText(2, QCoreApplication::translate("smollwidget", "17\345\200\215", nullptr));
        comboBox->setItemText(3, QCoreApplication::translate("smollwidget", "17.5\345\200\215", nullptr));
        comboBox->setItemText(4, QCoreApplication::translate("smollwidget", "18\345\200\215", nullptr));
        comboBox->setItemText(5, QCoreApplication::translate("smollwidget", "18.5\345\200\215", nullptr));
        comboBox->setItemText(6, QCoreApplication::translate("smollwidget", "19\345\200\215", nullptr));
        comboBox->setItemText(7, QCoreApplication::translate("smollwidget", "19.5\345\200\215", nullptr));
        comboBox->setItemText(8, QCoreApplication::translate("smollwidget", "20\345\200\215", nullptr));
        comboBox->setItemText(9, QCoreApplication::translate("smollwidget", "20.5\345\200\215", nullptr));
        comboBox->setItemText(10, QCoreApplication::translate("smollwidget", "21\345\200\215", nullptr));
        comboBox->setItemText(11, QCoreApplication::translate("smollwidget", "21.5\345\200\215", nullptr));
        comboBox->setItemText(12, QCoreApplication::translate("smollwidget", "22\345\200\215", nullptr));
        comboBox->setItemText(13, QCoreApplication::translate("smollwidget", "22.5\345\200\215", nullptr));
        comboBox->setItemText(14, QCoreApplication::translate("smollwidget", "23\345\200\215", nullptr));
        comboBox->setItemText(15, QCoreApplication::translate("smollwidget", "23.5\345\200\215", nullptr));

        label->setText(QCoreApplication::translate("smollwidget", "\344\270\262\345\217\243\351\200\211\346\213\251\357\274\232", nullptr));
        PortBox->setItemText(0, QCoreApplication::translate("smollwidget", "\345\210\267\346\226\260", nullptr));

#if QT_CONFIG(tooltip)
        PortBox->setToolTip(QCoreApplication::translate("smollwidget", "\344\270\262\345\217\243\351\200\211\346\213\251", nullptr));
#endif // QT_CONFIG(tooltip)
        pushButton_2->setText(QCoreApplication::translate("smollwidget", "\345\274\200\345\247\213\346\224\276\345\244\247", nullptr));
        OpenSerialButton->setText(QCoreApplication::translate("smollwidget", "\346\211\223\345\274\200\344\270\262\345\217\243", nullptr));
        pushButton->setText(QCoreApplication::translate("smollwidget", "\345\274\200\345\247\213\350\201\232\347\204\246", nullptr));
    } // retranslateUi

};

namespace Ui {
    class smollwidget: public Ui_smollwidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SMOLLWIDGET_H
