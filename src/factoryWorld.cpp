#include "factoryWorld.hpp"

namespace FactoryWorld {
  /** \class Machine
   * \brief   Contains information of production line.
   *
   * \fn Machine::capable
   * \brief   if the production line can produce this type represented by index.
   *
   * \fn explicit Machine::Machine(std::vector<Float> capability, Float readyTime)
   * \brief   construct product line given information.
   * \param[in]   capability   products per unit time, i.e. speed of production, each product type.
   * \param[in]   readyTime    No product can be scheduled before this time.
   *
   * \fn bool Machine::capable(Integral typeIndex) const
   * \brief   if the line can produce this product by index.
   * \param[in]   typeIndex   product index.
   *
   * \fn Float Machine::produceTime(Integral typeIndex, Integral numProduct) const
   * \brief   the time needed to produce 1 unit of some product.
   * \param[in]   typeIndex   product index.
   * \param[in]   numProduct  the number of products.
   *
   * \fn const std::vector<bool> & Machine::getCapableProduct() const
   * \brief   returns the boolean vector indicating product capabilities.
   *
   * \fn const std::vector<Float> & Machine::getCapability() const
   * \brief   returns the product per unit time of all products.
   *
   */

  /** \class RelationOfProducts
   * \brief   This class contains the relational information between products.
   *
   * This is the Generalization of Bill of Material(BOM) model, with the following types of relation:
   * - dependency between two products, i.e., one product precedes another.
   * - mutex between two products, i.e., enforce time interval between two products.
   *
   * \fn   explicit RelationOfProducts::RelationOfProducts(MatrixXd bom, MatrixTime gap)
   * Checking of constraints will be done inside:
   * - both bom and gap must be square.
   * - no negative value in bom and gap
   * -
   * \param[in]   MatrixXd   bom dependency matrix, number of products to produce another.
   * \param[in]   MatrixTime time gap between two products.
   */

  //using MatrixXd = Eigen::MatrixXd;
  //using MatrixB = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;
  /**
   * Process one order per line basis
   * productTypeSize for checking
   */
  template <typename InputStream>
  inline Order processOrder(InputStream &inputStream,
                            Integral productTypeSize) {
    LOG(WARNING)
      << "Now day -> hours is multipy by 10, assmuing 10 hours per day";
    // how many end products per order
    Integral typePerOrder;
    inputStream >> typePerOrder;
    if (!inputStream) {
      LOG(FATAL) << "Product size should be positive integer";
    }
    CHECK_GT(typePerOrder, 0);
    std::vector<Integral> productType(typePerOrder);
    std::vector<Integral> productQuan(typePerOrder);
    TimeUnit materialDate;
    TimeUnit dueTime;
    Integral clientID;
    for (Integral i = 0; i < typePerOrder; ++ i) {
      inputStream >> productQuan[i] >> productType[i];
      CHECK_GE(productQuan[i], 0);
      CHECK_GE(productType[i], 0);
      CHECK_LT(productType[i], productTypeSize);
    }

    inputStream >> materialDate >> dueTime >> clientID;
    return Order(std::move(productQuan), std::move(productType),
                 dueTime * 10, clientID, materialDate * 10);
  }

  /*
   * Process one bom line
   * productTypeSize for checking
   */
  template <typename InputStream>
  inline void processBOM(InputStream &inputStream,
                         Eigen::MatrixXd &bom,
                         Integral productTypeSize) {
    // product type and its predecessor
    Integral productType, dependentType, sizeOfTypes;
    Float dependentSize;
    inputStream >> productType >> sizeOfTypes;
    CHECK_GE(productType, 0);
    CHECK_LT(productType, productTypeSize);
    for (Integral i = 0; i < sizeOfTypes; ++ i) {
      inputStream >> dependentSize >> dependentType;
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
    std::vector<Float> capability(productTypeSize, 0.0);
    for (Integral i = 0; i < productTypeSize; ++ i) {
      if (!(inputStream >> capability[i]))
        LOG(FATAL) << "bad machine format";
    }
    Float readyTime;
    if (!(inputStream >> readyTime)) {
      LOG(FATAL) << "Machine ready time not found";
    }
    return Machine(capability, readyTime);
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
    if (!inputStream)
      LOG(FATAL) << "Size of product type should be integer";

    // process BOM
    Eigen::MatrixXd bom(productTypeSize, productTypeSize);
    Eigen::MatrixXd gap(productTypeSize, productTypeSize);
    for (Integral i = 0; i < productTypeSize; ++ i) {
      processBOM(inputStream, bom, productTypeSize);
    }

    // for (Integral i = 0; i < productTypeSize; ++ i) {
    //   // since these two are basically the same format
    //   processBOM(inputStream, gap, productTypeSize);
    // }
    gap.setConstant(0.0);
    LOG(WARNING) << "gap is now unimplemented, all default to 0";
    bom__ = RelationOfProducts(bom, gap);
    LOG(INFO) << productTypeSize << " products added";

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
    LOG(INFO) << machineSize << " machines added";

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
    LOG(INFO) << orderSize << " orders added";
  }

  std::ostream &operator<< (std::ostream &out, const Order &order) {
    const auto & productQuan = order.getProductQuan();
    const auto & productType = order.getProductType();
    const auto dueTime = order.getDueTime();
    const auto clientID = order.getClientID();
    const auto materialDate = order.getMaterialDate();

    out << "client " << clientID << " order ";
    IndexType size = productType.size();
    for (IndexType i = 0; i < size; ++ i) {
      out << productQuan[i] << " of product "
          << productType[i] << '\n';
    }
    out << "Due at " << dueTime << '\n';
    out << "Material arrived at " << materialDate << '\n';
    return out;
  }

}
