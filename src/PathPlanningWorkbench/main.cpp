#include "WorkbenchMainWindow.h"

#include <QApplication>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    path_planning_workbench::WorkbenchMainWindow window;
    window.show();

    return app.exec();
}

