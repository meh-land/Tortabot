import os

counter = 0
name=f'test_{counter:03}.txt'
while name in os.listdir("test"):
    print("True")
    counter+=1
    name=f'test_{counter:03}.txt'

f = open(f"test/{name}", "w")
f.write("Now the file has more sdds content!")
f.close()