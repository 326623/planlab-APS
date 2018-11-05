#include "factoryWorld.hpp"

namespace FactoryWorld {
  /**
   * Process one order per line basis
   * productTypeSize for checking
   */
  template <typename InputStream>
  inline Order processOrder(InputStream &inputStream,
                            Integral productTypeSize) {
    // how many end products per order
    Integral typePerOrder;
    inputStream >> typePerOrder;
    if (!inputStream) {
      LOG(FATAL) << "Product size should be positive integer";
    }
    CHECK_GE(typePerOrder, 0);
    CHECK_LT(typePerOrder, productTypeSize);
    std::vector<Integral> productType(typePerOrder);
    std::vector<Integral> productQuan(typePerOrder);
    TimeUnit materialDate;
    TimeUnit dueTime;
    Integral clientID;
    for (Integral i = 0; i < productType; ++ i) {
      inputStream >> productType[i] >> productQuan[i];
      CHECK_EQ(productType[i], 0);
      CHECK_EQ(productType[i], 0);
    }

    inputStream >> materialDaterawTime >> dueTime >> clientID;
    return Order(std::move(productQuan), std::move(productType),
                 dueTime, clientID, materialDate);
  }

  /*
   * Process one bom line
   * productTypeSize for checking
   */
  template <typename InputStream>
  inline void processBOM(InputStream &inputStream,
                    MatrixXd &bom,
                    Integral productTypeSize) {
    std::string lineBuffer;
    if (!std::getline(inputStream, lineBuffer))
      LOG(FATAL) << "Out of line to process on BOM!";
    std::stringstream lineStream(lineBuffer);
    // product type and its predecessor
    Integral productType, dependentType, dependentSize;
    lineStream >> productType;
    CHECK_GE(productType, 0);
    CHECK_LT(productType, productTypeSize);
    while (lineStream >> dependentType >> dependentSize) {
      CHECK_GE(dependentType, 0);
      CHECK_LT(dependentType, productTypeSize);
      // size must be larger than zero, otherwise no dependency
      CHECK_GT(dependentSize, 0);
      bom(productType, productTypeSize) = dependentSize;
    }
    if (!lineStream)
      LOG(FATAL) << "Bad format in bill of material";
  }

  /*
   * Process one machine capability per line
   * InputStream for streaming from possibly files
   * machineSize, productTypeSize for checking
   * Assume input is of a matrix form
   */
  template <typename InputStream>
  inline Machine processMachine(InputStream &inputStream,
                                Integral machineSize,
                                Integral productTypeSize) {
    std::string lineBuffer;
    if (!std::getline(inputStream, lineBuffer))
      LOG(FATAL) << "Out of line to Process on machine";
    stringstream stream(lineBuffer);
    for (Integral i = 0; i < productTypeSize; ++ i) {

    }
  }

  void Factory::load(const std::string &filename) {
    // have to assume that it haven't been loaded
    ifstream inputStream(filename);
    // handle first row
    inputStream >> tardyCost_ >> earlyCost_ >> idleCost_;
    if (!inputStream)
      LOG(FATAL) << "Costs should be floating numbers";

    // How many product type are there
    Integral productTypeSize;
    inputStream >> productTypeSize;
    CHECK_GE(productTypeSize, 0);
    if (!inputStream)
      LOG(FATAL) << "Size of product type should be integer";

    // process BOM
    MatrixXd bom(productTypeSize, productTypeSize);
    for (Integral i = 0; i < productTypeSize; ++ i) {
      processBOM(inputStream, bom, productTypeSize);
    }
    bom__ = BillOfMaterial(bom);

    // process machines
    Integral machineSize;
    inputStream >> machineSize;
    if (!inputStream)
      LOG(FATAL) << "machine size should be positive integer";
    CHECK_GT(machineSize, 0);

    machines__.resize(machineSize);
    for (Integral i = 0; i < machineSize; ++ i) {
      machines__[i] = processMachine(inputStream,
                                     machineSize,
                                     productTypeSize);
    }

    // process orders
    Integral orderSize, clientSize;
    inputStream >> orderSize >> clientSize;
    if (!inputStream)
      LOG(FATAL) << "order, client size should be positive integer";
    CHECK_GT(orderSize, 0);
    CHECK_GT(clientSize, 0);

    orders__.resize(orderSize);
    for (Integral i = 0; i < orderSize; ++ i) {
      orders__[i] = processOrder(inputStream, productTypeSize);
    }
  }
}
