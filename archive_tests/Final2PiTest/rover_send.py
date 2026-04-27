Last login: Sat Apr 18 14:07:15 on ttys000
sally@SallyMBP2 ~ % ssh-keygen -R 192.168.8.224
# Host 192.168.8.224 found: line 10
/Users/sally/.ssh/known_hosts updated.
Original contents retained as /Users/sally/.ssh/known_hosts.old
sally@SallyMBP2 ~ % ssh udmrt@192.168.8.224
kex_exchange_identification: read: Connection reset by peer
Connection reset by 192.168.8.224 port 22
sally@SallyMBP2 ~ % ssh udmrt@192.168.8.224
kex_exchange_identification: Connection closed by remote host
Connection closed by 192.168.8.224 port 22
sally@SallyMBP2 ~ % ssh-keygen -R 192.168.8.224
Host 192.168.8.224 not found in /Users/sally/.ssh/known_hosts
sally@SallyMBP2 ~ % ssh udmrt@192.168.8.224
kex_exchange_identification: read: Connection reset by peer
Connection reset by 192.168.8.224 port 22
sally@SallyMBP2 ~ % ssh udmrt@192.168.8.224
The authenticity of host '192.168.8.224 (192.168.8.224)' can't be established.
ED25519 key fingerprint is SHA256:2U58pLqOb9he2+lvmh/Pk7HzeBL7+VY5dCfuyiLuKo8.
This key is not known by any other names.
Are you sure you want to continue connecting (yes/no/[fingerprint])? yes
Warning: Permanently added '192.168.8.224' (ED25519) to the list of known hosts.
udmrt@192.168.8.224's password: 
Linux udmrtPi7Rover 6.12.75+rpt-rpi-2712 #1 SMP PREEMPT Debian 1:6.12.75-1+rpt1 (2026-03-11) aarch64

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
udmrt@udmrtPi7Rover:~ $ sudo raspi-config nonint do_serial_hw 0
[sudo] password for udmrt: 
udmrt@udmrtPi7Rover:~ $ sudo raspi-config nonint do_serial_cons 1
udmrt@udmrtPi7Rover:~ $ sudo apt-get update && sudo apt-get install python3-serial -y
Hit:1 http://deb.debian.org/debian trixie InRelease
Get:2 http://deb.debian.org/debian trixie-updates InRelease [47.3 kB]
Get:3 http://deb.debian.org/debian-security trixie-security InRelease [43.4 kB]
Get:4 http://deb.debian.org/debian-security trixie-security/main armhf Packages [119 kB]
Get:5 http://deb.debian.org/debian-security trixie-security/main arm64 Packages [124 kB]               
Get:6 http://deb.debian.org/debian-security trixie-security/main Translation-en [78.5 kB]              
Get:7 http://archive.raspberrypi.com/debian trixie InRelease [54.9 kB]                
Get:8 http://archive.raspberrypi.com/debian trixie/main arm64 Packages [424 kB]
Get:9 http://archive.raspberrypi.com/debian trixie/main armhf Packages [419 kB]
Fetched 1219 kB in 2s (743 kB/s) 
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
Fetched 88.2 kB in 0s (435 kB/s)    
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
udmrt@udmrtPi7Rover:~ $ sudo reboot
udmrt@udmrtPi7Rover:~ $ Connection to 192.168.8.224 closed by remote host.
Connection to 192.168.8.224 closed.
sally@SallyMBP2 ~ % nano rover_send.py

  UW PICO 5.09                                File: rover_send.py                                   

import serial
import time

# Open the port at 9600 speed
ser = serial.Serial('/dev/serial0', 9600, timeout=1)

print("ROVER: Sending heist code...")
ser.write(b"OPEN_SESAME\n") # The 'b' is for bytes

# Logic to wait for a reply for 3 seconds
start_time = time.time()
while (time.time() - start_time) < 3:
    if ser.in_waiting > 0:
        reply = ser.readline().decode('utf-8').strip()
        print(f"VAULT REPLIED: {reply}")
        break

    # CPU breather
    time.sleep(0.01)
else:
    print("TIMEOUT: No response from Vault.")

ser.close()

































                                         [ Wrote 23 lines ]                                         
^G Get Help     ^O WriteOut     ^R Read File    ^Y Prev Pg      ^K Cut Text     ^C Cur Pos      
^X Exit         ^J Justify      ^W Where is     ^V Next Pg      ^U UnCut Text   ^T To Spell     
