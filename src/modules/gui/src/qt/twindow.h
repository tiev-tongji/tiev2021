#ifndef TWINDOW_H
#define TWINDOW_H
#include <QMainWindow>

class QAction;
class QListWidget;
class QMenu;
class QTextEdit;

class TWindow : public QMainWindow {
    Q_OBJECT

public:
    TWindow();

private slots:
    void newLetter();
    void save();
    void print();
    void undo();
    void about();
    void insertCustomer(const QString& customer);
    void addParagraph(const QString& paragraph);

private:
    void createActions();
    void createStatusBar();
    void createDockWindows();

    QTextEdit* textEdit;
    QWidget*   cameraWidget;
    QWidget*   infoWidget;

    QMenu* viewMenu;
};

#endif
