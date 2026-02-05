#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

struct Client {
  std::string account;
  std::string name;
  std::string weigth;
  std::string age;
};

int compare (const void * a, const void * b) {
  return ( *(int*)a - *(int*)b );
}

int main() {

  std::cout<<"\n\n";
  std::cout<<"Quick sort in C++\n\n";
  std::ifstream inFile("registration.txt");

  const int size = 5;
  int i = 0;
  Client clients[size];
  int accs[size];

  if (inFile.is_open()) {
    std::string line;
    while(std::getline(inFile,line)) {

      std::stringstream ss(line);
      std::getline(ss,clients[i].account,',');
      std::cout<<"\""<<clients[i].account<<"\"";

      std::getline(ss,clients[i].name,',');
      std::cout<<", \""<<clients[i].name<<"\"";

      std::getline(ss,clients[i].weigth,',');
      std::cout<<", \""<<clients[i].weigth<<"\"";

      std::getline(ss,clients[i].age,',');
      std::cout<<", \""<<clients[i].age<<"\"";

      accs[i] = std::stoi(clients[i].account);

      std::cout<<"\n";
      i = i + 1;
    }
  }

  std::cout<<"\n\n";
  std::cout<<"No sorted:\n";
  for (i=0; i<size; i++) printf ("%d ",accs[i]);

  std::cout<<"\n\n";
  std::cout<<"Quick sorted:\n";
  qsort (accs, size, sizeof(int), compare);
  for (i=0; i<size; i++) printf ("%d ",accs[i]);
  std::cout<<"\n\n";

  return 0;
}

