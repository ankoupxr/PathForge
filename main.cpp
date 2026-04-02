#include <QApplication>
#include <QStyleFactory>
#include "MainWindow.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    // 设置应用程序信息
    QApplication::setApplicationName("PathForge");
    QApplication::setApplicationVersion("1.0.0");
    QApplication::setOrganizationName("PathForge");

    // 设置 Fusion 风格（现代化外观）
    QApplication::setStyle(QStyleFactory::create("Fusion"));

    // 设置深色主题调色板
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);
    QApplication::setPalette(darkPalette);

    // 创建并显示主窗口
    MainWindow mainWindow;
    mainWindow.show();

    return QApplication::exec();
}
