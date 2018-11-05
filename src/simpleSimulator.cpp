#include "randomOrderGen.hpp"

int main() {
  //using namespace ;
  // TODO: should fix
  // sadly can't use namespace, a conflict of class and namespace
  std::pair<int, int> productSizeRange = {0, 1000};
  auto numClients = 18;
  auto productTypeSize = 300;
  Simulator::OrderGenerator<int> orderGen(productTypeSize,
                                          {1e-5, 1e-4, 1e-3, 1e-2, 1e-1},
                                          numClients,
                                          productSizeRange);

  Simulator::Simulator<
    Simulator::Order<int>,
    Simulator::OrderGenerator<int>> simulator(2);
  std::vector<Simulator::Order<int>> randomOrders;
  for (int i = -30; i <= 0; ++ i) {
    orderGen.setStartTime(i);
    auto orders = simulator.simulate(orderGen);
    randomOrders.insert(randomOrders.end(), orders.begin(), orders.end());
  }

  auto orderSize = randomOrders.size();
  std::cout << orderSize << ' ' << numClients << '\n';

  for (const auto &order : randomOrders)
    std::cout << order << '\n';
}
