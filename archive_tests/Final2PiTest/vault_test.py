Last login: Fri Apr 17 22:15:40 on ttys000
sally@SallyMBP2 ~ % ssh udmrt@192.168.8.233
The authenticity of host '192.168.8.233 (192.168.8.233)' can't be established.
ED25519 key fingerprint is SHA256:wzKQqJglpMIL9PxuWBTwhxxRCl0RzTH/BF94YdZCs+E.
This key is not known by any other names.
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
Warning: Permanently added '192.168.8.233' (ED25519) to the list of known hosts.
ssh_dispatch_run_fatal: Connection to 192.168.8.233 port 22: Broken pipe
sally@SallyMBP2 ~ % ssh-keygen -R 192.168.8.233
# Host 192.168.8.233 found: line 7
/Users/sally/.ssh/known_hosts updated.
Original contents retained as /Users/sally/.ssh/known_hosts.old
sally@SallyMBP2 ~ % ssh udmrt@192.168.8.233
The authenticity of host '192.168.8.233 (192.168.8.233)' can't be established.
ED25519 key fingerprint is SHA256:wzKQqJglpMIL9PxuWBTwhxxRCl0RzTH/BF94YdZCs+E.
This key is not known by any other names.
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
Warning: Permanently added '192.168.8.233' (ED25519) to the list of known hosts.
udmrt@192.168.8.233's password: 
Permission denied, please try again.
udmrt@192.168.8.233's password: 
Linux udmrtPi7Vault 6.12.75+rpt-rpi-2712 #1 SMP PREEMPT Debian 1:6.12.75-1+rpt1 (2026-03-11) aarch64

The programs included with the Debian GNU/Linux system are free software;
the exact distribution terms for each program are described in the
individual files in /usr/share/doc/*/copyright.

Debian GNU/Linux comes with ABSOLUTELY NO WARRANTY, to the extent
permitted by applicable law.
_____________________________________________________________________
WARNING! Your environment specifies an invalid locale.
 The unknown environment variables are:
   LC_CTYPE=UTF-8 LC_ALL=
 This can affect your user experience significantly, including the
 ability to manage packages. You may install the locales by running:

 sudo dpkg-reconfigure locales

 and select the missing language. Alternatively, you can install the
 locales-all package:

 sudo apt-get install locales-all

To disable this message for all users, run:
   sudo touch /var/lib/cloud/instance/locale-check.skip
_____________________________________________________________________

-bash: warning: setlocale: LC_CTYPE: cannot change locale (UTF-8): No such file or directory
-bash: warning: setlocale: LC_CTYPE: cannot change locale (UTF-8): No such file or directory
-bash: warning: setlocale: LC_CTYPE: cannot change locale (UTF-8): No such file or directory
-bash: warning: setlocale: LC_CTYPE: cannot change locale (UTF-8): No such file or directory
-bash: warning: setlocale: LC_CTYPE: cannot change locale (UTF-8): No such file or directory
-bash: warning: setlocale: LC_CTYPE: cannot change locale (UTF-8): No such file or directory
-bash: warning: setlocale: LC_CTYPE: cannot change locale (UTF-8): No such file or directory
-bash: warning: setlocale: LC_CTYPE: cannot change locale (UTF-8): No such file or directory
udmrt@udmrtPi7Vault:~ $ sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_serial_cons 1
[sudo] password for udmrt: 
udmrt@udmrtPi7Vault:~ $ sudo apt-get update
sudo apt-get install python3-serial -y
Hit:1 http://deb.debian.org/debian trixie InRelease
Get:2 http://deb.debian.org/debian trixie-updates InRelease [47.3 kB]
Get:3 http://deb.debian.org/debian-security trixie-security InRelease [43.4 kB] 
Get:4 http://deb.debian.org/debian-security trixie-security/main arm64 Packages [124 kB]     
Get:5 http://deb.debian.org/debian-security trixie-security/main armhf Packages [119 kB]
Get:6 http://deb.debian.org/debian-security trixie-security/main Translation-en [78.5 kB]
Get:7 http://archive.raspberrypi.com/debian trixie InRelease [54.9 kB]                
Get:8 http://archive.raspberrypi.com/debian trixie/main arm64 Packages [424 kB]
Get:9 http://archive.raspberrypi.com/debian trixie/main armhf Packages [419 kB]
Fetched 1219 kB in 1s (897 kB/s) 
Reading package lists... Done
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
Suggested packages:
  python3-wxgtk3.0 | python3-wxgtk
The following NEW packages will be installed:
  python3-serial
0 upgraded, 1 newly installed, 0 to remove and 2 not upgraded.
Need to get 88.2 kB of archives.
After this operation, 469 kB of additional disk space will be used.
Get:1 http://deb.debian.org/debian trixie/main arm64 python3-serial all 3.5-2 [88.2 kB]
Fetched 88.2 kB in 0s (665 kB/s)       
perl: warning: Setting locale failed.
perl: warning: Please check that your locale settings:
	LANGUAGE = (unset),
	LC_ALL = (unset),
	LC_CTYPE = "UTF-8",
	LC_NUMERIC = (unset),
	LC_COLLATE = (unset),
	LC_TIME = (unset),
	LC_MESSAGES = (unset),
	LC_MONETARY = (unset),
	LC_ADDRESS = (unset),
	LC_IDENTIFICATION = (unset),
	LC_MEASUREMENT = (unset),
	LC_PAPER = (unset),
	LC_TELEPHONE = (unset),
	LC_NAME = (unset),
	LANG = "en_GB.UTF-8"
    are supported and installed on your system.
perl: warning: Falling back to a fallback locale ("en_GB.UTF-8").
locale: Cannot set LC_CTYPE to default locale: No such file or directory
locale: Cannot set LC_ALL to default locale: No such file or directory
Selecting previously unselected package python3-serial.
(Reading database ... 66247 files and directories currently installed.)
Preparing to unpack .../python3-serial_3.5-2_all.deb ...
Unpacking python3-serial (3.5-2) ...
Setting up python3-serial (3.5-2) ...
udmrt@udmrtPi7Vault:~ $ nano vault_test.py

  GNU nano 8.4                                vault_test.py                                         
import serial # Toolbox for the pins
import time   # Toolbox for the clock

ser = serial.Serial('/dev/serial0', 9600, timeout=1) 

print("VAULT: Waiting...")
try:
    while True:
        if ser.in_waiting > 0:
            # Read and clean the incoming data
            data = ser.readline().decode('utf-8').strip()
            print(f"ALARM: {data}")
            # Send a reply back to the Rover
            ser.write(b"ACCESS DENIED\n")
        
        # CPU breather - MUST be inside while, but outside if
        time.sleep(0.1) 
except KeyboardInterrupt:
    print("\nShutting down Vault...")
    ser.close()





































                                         [ Wrote 20 lines ]
^G Help       ^O Write Out  ^F Where Is   ^K Cut        ^T Execute    ^C Location   M-U Undo
^X Exit       ^R Read File  ^\ Replace    ^U Paste      ^J Justify    ^/ Go To Line M-E Redo
