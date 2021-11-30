# pcl_custom_functions
Some custom implementations of the Point Cloud Library functions that run faster then the default PCL implementation.

Tested with PCL-1.10 under Ubuntu 20.04.
```
$ sudo apt install libpcl-dev
```

To build the code: 
```
$ mkdir -p build/Release
$ cd build/Release
$ cmake -DCMAKE_BUILD_TYPE=Release ../../
$ make -j4
```

To run the main program: 
```
$ ./main
```
To run the programing with timings:
```
$ ./timing_main
```


