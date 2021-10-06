import QtQuick 2.14
import QtQuick.Window 2.14
import QtQuick.Controls 2.15

Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")

    Button {
        id: button
        x: 37
        y: 41
        text: qsTr("Button")
        checked: false
        checkable: false
        highlighted: false
        onClicked:console.log("clicked")


    }

    TextArea {
        id: textArea
        x: 37
        y: 147
        width: 440
        height: 186
        text: "sadf"
        placeholderText: qsTr("Text Area")
    }


}
