import machine
import ujson
def read_commands():
    with open('commands.json', 'r') as file2:
        data = ujson.load(file2)
        return data
def erase():
    with open ('commands.json', 'w') as file3:
        file3.write('')
print('Executing Commands')
def process_and_execute(commandlist):
    runcode = commandlist[10]
    if runcode != 38241232:
        erase()
        machine.reset()
    else:
        for command in commandlist:
            print(command)
            cid = int(command[:0])
            arg1 = int(command[1:2])
            arg2 = int(command[3:5])
            if cid == 1:
                forward(arg1, arg2)
            elif cid == 2:
                back(arg1, arg2)
            elif cid == 3:
                left(arg1, arg2)
            elif cid == 4:
                right(arg1, arg2)
        erase()
def boot2():
    commands = read_commands()
    cmd1 = commands['cmd1']
    cmd2 = commands['cmd2']
    cmd3 = commands['cmd3']
    cmd4 = commands['cmd4']
    cmd5 = commands['cmd5']
    cmd6 = commands['cmd6']
    cmd7 = commands['cmd7']
    cmd8 = commands['cmd8']
    cmd9 = commands['cmd9']
    cmd10 = commands['cmd10']
    runcode = commands['runcode']
    tobeexeucted = [cmd1, cmd2, cmd3, cmd4, cmd5, cmd6, cmd7, cmd8, cmd9, cmd10, runcode]
    process_and_execute(tobeexeucted)
