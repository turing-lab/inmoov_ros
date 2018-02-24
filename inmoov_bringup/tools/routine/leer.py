f = open("angles.txt","r")
while True :
	line = f.readline()
	print line
	if ("" == line):
    		print "file finished"
    		break