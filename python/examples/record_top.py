import subprocess


def save_top():
  with open("top_calls.txt", "w") as outfile:
    subprocess.call(r'top -d 5 -b | grep "load average" -A 15 > top_calls.txt', shell=True, stdout=outfile)



def save_ping():
  with open("ping_calls.txt", "w") as outfile:
    subprocess.call(r'ping 192.168.2.51 | while read pong; do echo "$(date): $pong"; done > ping_log.txt', shell=True, stdout=outfile)


save_top()