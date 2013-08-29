Main.exe		: 	Main.o  transform.o graphics.o algorithm.o astarsearch.o lpastarsearch.o
	g++ -Wl,-s -o Main.exe Main.o algorithm.o transform.o graphics.o astarsearch.o lpastarsearch.o

Main.o		:	Main.cpp graphics.h transform.h globalvariables.h astarsearch.h
	g++ -c -fpermissive -fconserve-space Main.cpp

algorithm.o		:	 algorithm.cpp algorithm.h
	g++ -c -fpermissive -fconserve-space algorithm.cpp	
	
transform.o		:	 transform.cpp transform.h
	g++ -c -fpermissive -fconserve-space transform.cpp	

graphics.o		:	 graphics.cpp graphics.h
	g++ -c -fpermissive -fconserve-space graphics.cpp

astarsearch.o		:	 astarsearch.cpp astarsearch.h
	g++ -c -fpermissive -fconserve-space astarsearch.cpp	

lpastarsearch.o	:	 LPAstarSearch.cpp LPAstarSearch.h
	g++ -c -fpermissive -fconserve-space LPAstarSearch.cpp	
