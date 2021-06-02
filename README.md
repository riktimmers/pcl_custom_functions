# pcl_custom_functions
Some custom implementations of the Point Cloud Library functions that run faster. 

Tested with PCL-1.10 under Ubuntu 20.
```
$ sudo apt install libpcl-dev
```

To build the code: 
```
$ mkdir Release
$ cd Release
$ cmake -DCMAKE_BUILD_TYPE=Release .. 
$ make
```

To run the test: 
```
$ ./main
```


