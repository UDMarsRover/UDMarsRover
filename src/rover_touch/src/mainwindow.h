#pragma once

#include <QMainWindow>
#include <memory>

class UiNode; // forward-declare (defined in touch_ui_node.h)

// Forward-declare the UI class that CMAKE_AUTOUIC will create
namespace Ui {
  class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT // <-- Required for signals/slots

public:
  MainWindow(QWidget *parent = nullptr);
  MainWindow(std::shared_ptr<UiNode> ui_node, QWidget *parent = nullptr);
  ~MainWindow();

private slots:
  void on_myButton_clicked(); // <-- A slot to handle the click

private:
  // This pointer will hold all your UI elements
  Ui::MainWindow *ui;
  std::shared_ptr<UiNode> ui_node_;
};