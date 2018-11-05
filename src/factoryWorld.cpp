#include "factoryWorld.hpp"

namespace FactoryWorld {
  //using MatrixXd = Eigen::MatrixXd;
  //using MatrixB = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;
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
    for (Integral i = 0; i < typePerOrder; ++ i) {
      inputStream >> productQuan[i] >> productType[i];
      CHECK_EQ(productType[i], 0);
      CHECK_EQ(productType[i], 0);
    }

    inputStream >> materialDate >> dueTime >> clientID;
    return Order(std::move(productQuan), std::move(productType),
                 dueTime, clientID, materialDate);
  }

  /*
   * Process one bom line
   * productTypeSize for checking
   */
  template <typename InputStream>
  inline void processBOM(InputStream &inputStream,
                         Eigen::MatrixXd &bom,
                         Integral productTypeSize) {
    std::string lineBuffer;
    if (!std::getline(inputStream, lineBuffer))
      LOG(FATAL) << "Out of line to process on BOM!";
    std::stringstream lineStream(lineBuffer);
    std::cout << lineBuffer << '\n';
    // product type and its predecessor
    Integral productType, dependentType;
    Float dependentSize;
    lineStream >> productType;
    CHECK_GE(productType, 0);
    CHECK_LT(productType, productTypeSize);
    while (lineStream >> dependentType) {
      if (!(lineStream >> dependentSize))
        LOG(FATAL) << "Bad format in bill of material";
      CHECK_GE(dependentType, 0);
      CHECK_LT(dependentType, productTypeSize);
      // size must be larger than zero, otherwise no dependency
      CHECK_GT(dependentSize, 0.0);
      bom(productType, dependentType) = dependentSize;
    }
  }

  /*
   * Process one machine capability per line
   * InputStream for streaming from possibly files
   * machineSize, productTypeSize for checking
   * Assume input is of a matrix form
   */
  template <typename InputStream>
  inline Machine processMachine(InputStream &inputStream,
                                Integral productTypeSize) {
    std::string lineBuffer;
    if (!std::getline(inputStream, lineBuffer))
      LOG(FATAL) << "Out of line to Process on machine";
    std::stringstream stream(lineBuffer);
    std::vector<Float> capability(productTypeSize);
    for (Integral i = 0; i < productTypeSize; ++ i) {
      stream >> capability[i];
    }
    return Machine(capability);
  }

  template <typename InputStream>
  inline void consumeOneEmptyLine(InputStream &inputStream) {
    std::string buffer;
    std::getline(inputStream, buffer);
    //while (buffer.empty() && std::getline(inputStream, buffer))
    //      { /* skip empty line*/ }
  }

  void Factory::load(const std::string &filename) {
    // have to assume that it haven't been loaded
    std::ifstream inputStream(filename);
    if (!inputStream.is_open())
      LOG(FATAL) << "Cannot open given file";
    // handle first row
    inputStream >> tardyCost_ >> earlyCost_ >> idleCost_;
    if (!inputStream)
      LOG(FATAL) << "Costs should be floating numbers";

    // How many product type are there
    Integral productTypeSize;
    inputStream >> productTypeSize;
    CHECK_GE(productTypeSize, 0);
    LOG(INFO) << "added " << productTypeSize << " products";
    if (!inputStream)
      LOG(FATAL) << "Size of product type should be integer";

    // process BOM
    Eigen::MatrixXd bom(productTypeSize, productTypeSize);
    for (Integral i = 0; i < productTypeSize; ++ i) {
      LOG(INFO) << i << '\n';
      processBOM(inputStream, bom, productTypeSize);
    }
    bom__ = RelationOfProducts(bom);

    // process machines
    Integral machineSize;
    inputStream >> machineSize;
    if (!inputStream)
      LOG(FATAL) << "machine size should be positive integer";
    CHECK_GT(machineSize, 0);

    machines__.resize(machineSize);
    for (Integral i = 0; i < machineSize; ++ i) {
      machines__[i] = processMachine(inputStream,
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
