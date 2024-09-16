# Tests

To compile the file `unit_tests.cpp` you need to have the `googletest` library installed. You can install it by running the following command:

```bash
sudo apt-get install libgtest-dev
```

To compile it, run the following commands:

```bash
g++ -std=c++17 unitary_tests.cpp ../code/base.cpp -lgtest -lgtest_main -pthread -fopenmp -o testBase
```
