#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

uint64_t fibonacci(uint64_t n, uint64_t* memo, size_t size) {
  if (n == 0) return 0;
  if (n == 1) return 1;
  if (n >= size) return -1;

  if (memo[n] != 0) return memo[n];

  uint64_t result = fibonacci(n - 1, memo, size) + fibonacci(n - 2, memo, size);
  memo[n] = result;
  return result;
}

int main() {
  size_t size = 20;
  uint64_t memo[20] = {0};

  for (size_t i = 0; i < size; i++) {
    printf("F(%zu) = %lu\n", i, fibonacci(i, memo, size));
  }

  return 0;
}

