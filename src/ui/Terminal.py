import shlex
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QTextEdit, QLineEdit
from PyQt6.QtCore import QProcess

class Terminal(QWidget):
    def __init__(self):
        super().__init__()

        self.terminalOut = QTextEdit()
        self.terminalOut.setReadOnly(True)

        self.terminalIn = QLineEdit()
        self.terminalIn.returnPressed.connect(self.run_command)  # Run command on Enter

        #Adds everything to the layouts
        layout = QVBoxLayout()
        layout.addWidget(self.terminalOut)
        layout.addWidget(self.terminalIn)
        self.setLayout(layout)

        #Starts a process to auto-update the output terminal
        self.process = QProcess()
        self.process.readyReadStandardOutput.connect(self.handle_stdout)
        self.process.readyReadStandardError.connect(self.handle_stderr)
        self.process.finished.connect(self.finished)

    def run_command(self): #Updates the output
        command = self.terminalIn.text().strip()
        self.terminalIn.clear()
        self.terminalOut.append(f"> {command}")
        
        args = shlex.split(command)
        if not args:
            return
        if len(args) == 1:
            program = "bash.exe"    # Change this for linux systems
            arguments = args[0]
        program = args[0]
        arguments = args[1:]
        
        self.process.start(program, arguments)

    def handle_stdout(self):
        output = self.process.readAllStandardOutput().data().decode()
        self.terminalOut.append(output)

    def handle_stderr(self):
        #Handles error output from the process.
        error = self.process.readAllStandardError().data().decode()
        print(error)
        self.terminalOut.append(f"Error: {error}")
    
    def finished(self):
        self.terminalOut.append("Process finished")