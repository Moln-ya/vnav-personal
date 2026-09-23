#include "random_vector.h"
#include <cstdlib>

RandomVector::RandomVector(int size, double max_val) {
  for (int i = 0; i < size; ++i) {
    double value =
        static_cast<double>(std::rand()) /
        static_cast<double>(RAND_MAX) * max_val;

    vect.push_back(value);
  }
}

void RandomVector::print() {
  for (std::size_t i = 0; i < vect.size(); ++i) {
    std::cout << vect[i];

    if (i + 1 < vect.size()) {
      std::cout << " ";
    }
  }

  std::cout << std::endl;
}

double RandomVector::mean() {
  if (vect.empty()) {
    return 0.0;
  }

  double sum = 0.0;

  for (std::size_t i = 0; i < vect.size(); ++i) {
    sum += vect[i];
  }

  return sum / static_cast<double>(vect.size());
}

double RandomVector::max() {
  if (vect.empty()) {
    return 0.0;
  }

  double max_value = vect[0];

  for (std::size_t i = 1; i < vect.size(); ++i) {
    if (vect[i] > max_value) {
      max_value = vect[i];
    }
  }

  return max_value;
}

double RandomVector::min() {
  if (vect.empty()) {
    return 0.0;
  }

  double min_value = vect[0];

  for (std::size_t i = 1; i < vect.size(); ++i) {
    if (vect[i] < min_value) {
      min_value = vect[i];
    }
  }

  return min_value;
}

void RandomVector::printHistogram(int bins) {
  if (bins <= 0 || vect.empty()) {
    return;
  }

  std::vector<int> histogram(bins, 0);

  double min_value = min();
  double max_value = max();

  if (max_value == min_value) {
    histogram[0] = static_cast<int>(vect.size());
  } else {
    double bin_width =
        (max_value - min_value) / static_cast<double>(bins);

    for (std::size_t i = 0; i < vect.size(); ++i) {
      int bin =
          static_cast<int>((vect[i] - min_value) / bin_width);

      // The maximum value lies exactly on the upper boundary.
      // Put it into the final bin.
      if (bin >= bins) {
        bin = bins - 1;
      }

      if (bin < 0) {
        bin = 0;
      }

      ++histogram[bin];
    }
  }

  int max_count = histogram[0];

  for (int i = 1; i < bins; ++i) {
    if (histogram[i] > max_count) {
      max_count = histogram[i];
    }
  }

  // Print histogram vertically from top to bottom.
  for (int level = max_count; level > 0; --level) {
    for (int i = 0; i < bins; ++i) {
      if (histogram[i] >= level) {
        std::cout << "***";
      } else {
        std::cout << "   ";
      }

      if (i + 1 < bins) {
        std::cout << " ";
      }
    }

    std::cout << std::endl;
  }
}
