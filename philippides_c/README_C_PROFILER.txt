README profiler C 

 porfiler sur le mbs en C 
1 installer kcachergrind : sudo snap install kcachegrind
2 installer valgrind 
3 valgrind --tool=callgrind ./exe_philippides_c
 dans le terminal il y aura un nombre a 4 chifres xxxx
4 regarder ce qu'il est resorti dans le valgrind avec : kcachegrind callgrind.out.xxxx
