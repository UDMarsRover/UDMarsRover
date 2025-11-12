#include "mainwindow.h"
#include "touch_ui_node.h"
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

// Simple implementation wiring up a button to the slot.
MainWindow::MainWindow(QWidget *parent)
  : QMainWindow(parent), ui(nullptr)
{
  // Create UiNode instance (ROS node + publisher). Spun externally in main.
  ui_node_ = std::make_shared<UiNode>();
  // For now, build a minimal UI manually (no .ui file yet).
  auto *central = new QWidget(this);
  auto *layout = new QVBoxLayout(central);
  auto *button = new QPushButton("Press Me", central);
  layout->addWidget(button);
  central->setLayout(layout);
  setCentralWidget(central);



  // Connect button click to slot.
  connect(button, &QPushButton::clicked, this, &MainWindow::on_myButton_clicked);
}

MainWindow::MainWindow(std::shared_ptr<UiNode> ui_node, QWidget *parent)
  : QMainWindow(parent), ui(nullptr), ui_node_(std::move(ui_node))
{
  auto *central = new QWidget(this);
  auto *layout = new QVBoxLayout(central);
  auto *button = new QPushButton("Press Me", central);
  layout->addWidget(button);
  central->setLayout(layout);
  setCentralWidget(central);
  connect(button, &QPushButton::clicked, this, &MainWindow::on_myButton_clicked);
}

MainWindow::~MainWindow() = default;

void MainWindow::on_myButton_clicked() {
  if (ui_node_) {
    ui_node_->publish_button_press("Press Me clicked");
  }
}
