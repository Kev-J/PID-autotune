#pragma once

#include <QSpinBox>
#include <QtCore/QObject>

// Thanks to ChrisW67 from the Qtproject forum: 
// http://qt-project.org/forums/viewthread/24037
class OddSpinBox : public QSpinBox {
    // Q_OBJECT
public:
    OddSpinBox(QWidget *parent = 0)
        : QSpinBox(parent)
    {
        setSingleStep(2);
    }

	//virtual ~OddSpinBox() = default;

    protected:
    int valueFromText(const QString &text) const
    {
        int textIntLength = text.length() - prefix().length() - suffix().length();
        QString textInt = text.mid(prefix().length(), textIntLength);
        int value = textInt.toInt();
        if (value % 2 != 0)
            value--;

        value = qMax(value, minimum());
        value = qMin(value, maximum());

        return value;
    }
};
