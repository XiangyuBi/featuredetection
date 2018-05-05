#ifndef HEAT_HEAT_HH__
#define HEAT_HEAT_HH__

#include <Eigen/Core>
#include "pclbo.h"

namespace heat {

template <typename PointT>
class HeatKernelSignature {
public:
    typedef std::shared_ptr<HeatKernelSignature<PointT> > Ptr;
    typedef std::shared_ptr<std::vector<double> > HeatSignature;
    virtual ~HeatKernelSignature() {}
    void setInputCloud(const typename pcl::PointCloud<PointT>::Ptr& cloud) {
        _cloud = cloud;
    }

    void setEigenValues(const Eigen::VectorXd& values) {
        _eigenvalue = values;
    }
    void setEigenFunctions(const Eigen::MatrixXd& vectors) {
        _eigenfunctions = vectors;
    }
    HeatSignature compute(const int x, const double t, const int n_functions = 200);

private:
    Eigen::MatrixXd _eigenfunctions;
    Eigen::VectorXd _eigenvalue;
    typename pcl::PointCloud<PointT>::Ptr _cloud;

}; // class HeatKernelSignature

} // namespace heat

template <typename PointT>
typename heat::HeatKernelSignature<PointT>::HeatSignature 
heat::HeatKernelSignature<PointT>::compute(const int x, const double t, const int n_functions) {

    HeatSignature hs(new std::vector<double>(_cloud->size()));

    for (int y = 0; y < _cloud->size(); y++) {

        double sum = 0.0;
        for (int j = 0; j < n_functions; j++) {
            double lambda = _eigenvalue(j);
            Eigen::VectorXd psi = _eigenfunctions.col(j);
            sum += exp(-lambda * t) * psi(x) * psi(y); 
        }

        hs->at(y) = sum;
    }

    return hs;
}
#endif // HEAT_HEAT_HH__