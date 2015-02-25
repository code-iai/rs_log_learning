#include <iostream>
#include <mlpack/core.hpp>
#include <mlpack/methods/neighbor_search/neighbor_search.hpp>

using namespace mlpack;
using namespace mlpack::neighbor; // NeighborSearch and NearestNeighborSort

int main()
{
	// Our dataset matrix, which is column-major.
    arma::mat data;
    data::Load("data.csv", data, true);

    AllkNN a(data);

    // The matrices we will store output in.
    arma::Mat<size_t> resultingNeighbors;
    arma::mat resultingDistances;

    a.Search(1, resultingNeighbors, resultingDistances);

    data.print();

    // Write each neighbor and distance using Log.
    for (size_t i = 0; i < resultingNeighbors.n_elem; ++i)
    {
    std::cout << "Nearest neighbor of point " << i << " is point "
        << resultingNeighbors[i] << " and the distance is " << resultingDistances[i] << std::endl;
    }

    std::cout << "==================================================================" << std::endl;

    arma::mat data2(2,2, arma::fill::zeros);

    data2(0,0) = 1;
    data2(0,1) = 3;

    data2.print();
    //data2.resize(3,3);
    data2.resize(1,1);
    data2.resize(1,1);

    data2(0,0) = 42;

    for(int i = 1; i <= 10; ++i)
    {
        data2.resize(1,i);
        std::cout << "setting: " << i-1 << std::endl;
        data2(0,i-1) = i;
    }
    std::cout << "-----------" << std::endl;
    data2.print();

    AllkNN b(data2);

    // The matrices we will store output in.
    arma::Mat<size_t> resultingNeighbors2;
    arma::mat resultingDistances2;

    b.Search(1, resultingNeighbors2, resultingDistances2);

    // Write each neighbor and distance using Log.
    for (size_t i = 0; i < resultingNeighbors2.n_elem; ++i)
    {
    std::cout << "Nearest neighbor of point " << i << " is point "
        << resultingNeighbors2[i] << " and the distance is " << resultingDistances2[i] << std::endl;
    }
}
