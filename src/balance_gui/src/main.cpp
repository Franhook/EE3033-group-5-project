#include <QApplication>
#include "balance_gui/main_window.hpp"

int main(int argc, char *argv[])
{
    // Support high-DPI displays (without these two lines, the UI appears very small on some high-resolution screens)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    // Initialize the Qt application instance
    QApplication a(argc, argv);

    // Instantiate the main window
    MainWindow w(argc, argv);
    
    // Set the window title and size
    w.setWindowTitle("Balance Robot Control Panel");
    w.show();

    // Enter Qt's main event loop (the backend QNode thread is already running ROS spin)
    return a.exec();
}
