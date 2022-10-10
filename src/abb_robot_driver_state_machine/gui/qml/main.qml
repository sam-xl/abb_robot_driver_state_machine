import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    visible: true
    width: 600
    height: 500
    title: "ABB Driver: State Machine Manager"

    ColumnLayout {
        width: parent.width
        height: parent.height

        Label {
            text: "State:"
            font.pixelSize: 24
        }
        RowLayout {
            Rectangle {
                Layout.preferredWidth: 100
                Layout.preferredHeight: 35
                color: 'azure'
                Label {
                    anchors.fill:parent
                    text: "Initialize"
                }
            }
            Rectangle {
                Layout.preferredWidth: 100
                Layout.preferredHeight: 35
                Label {
                    text: "Motors On"
                }
            }
            Rectangle {
                Layout.preferredWidth: 100
                Layout.preferredHeight: 35
                color: 'azure'
                Label {
                    text: "Idle"
                }
            }
            Rectangle {
                Layout.preferredWidth: 100
                Layout.preferredHeight: 35
                Text {
                    anchors.fill:parent
                    text: "Run EGM Session"
                }
            }
            Rectangle {
                Layout.preferredWidth: 100
                Layout.preferredHeight: 35
                Text {
                    text: "Run RAPID Program"
                }
            }
        }

        RowLayout {
            ColumnLayout {
                Text {
                    text: "EGM"
                    font.pixelSize: 24
                }
                RowLayout {
                    Button {
                        text: "Start"
                    }
                    Button {
                        text: "Stop"
                    }
                }
            }

            ColumnLayout {
                Text {
                    text: "RAPID"
                    font.pixelSize: 24
                }
                RowLayout {
                    Button {
                        text: "Start"
                    }
                    Button {
                        text: "Stop"
                    }
                }
            }
        }

        Item {
            Layout.fillHeight: true
        }
        Button {
            width: Layout.width
            text: "Exit"
        }
    }
}
