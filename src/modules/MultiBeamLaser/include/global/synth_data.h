#include <multibooster/multibooster.h>

using namespace Eigen;
using namespace std;


void swapNames(vector<string>* pvec, size_t i, size_t j) {
  vector<string>& vec = *pvec;
  string tmp = vec[i];
  vec[i] = vec[j];
  vec[j] = tmp;
}

void permuteNames(vector<string>* names) {
  for(size_t i=0; i<100; ++i) {
    size_t idx1 = rand() % names->size();
    size_t idx2 = rand() % names->size();
    swapNames(names, idx1, idx2);
  }
}

VectorXf* sampleFromMultidimGaussian(VectorXf mean, double stdev) {
  VectorXf* sample = new VectorXf;
  *sample = VectorXf::Zero(mean.rows());
  for(size_t i=0; i<(size_t)mean.rows(); ++i) {
    (*sample)(i) = sampleFromGaussian(stdev) + mean(i);
  }
  return sample;
}

class SynthDatasetGenerator { 
 public:
  size_t num_objs_;
  size_t num_feature_spaces_;
  size_t num_classes_;
  size_t num_dims_;
  size_t num_bg_;
  double variance_;
  double feature_probability_;
  bool labeled_;
  
  MultiBoosterDataset* generateDataset();
  SynthDatasetGenerator();
};

SynthDatasetGenerator::SynthDatasetGenerator() :
  num_objs_(100),
  num_feature_spaces_(4),
  num_classes_(3),
  num_dims_(2),
  num_bg_(500),
  variance_(1),
  feature_probability_(.75),
  labeled_(true)  
{
}

MultiBoosterDataset* SynthDatasetGenerator::generateDataset() {
  // -- Make names for classes and feature spaces.
  vector<string> feature_spaces;
  vector<string> classes;
  for(size_t i=0; i<num_feature_spaces_; ++i) {
    ostringstream oss;
    oss << "Feature_space_" << i;
    feature_spaces.push_back(oss.str());
  }
  for(size_t i=0; i<num_classes_; ++i) {
    ostringstream oss;
    oss << "Class_" << i;
    classes.push_back(oss.str());
  }
  
  // -- Get a random permutation of those.
  permuteNames(&feature_spaces);
  permuteNames(&classes);

  NameMapping cm(classes);
  NameMapping fsm(feature_spaces);

  // -- Randomly sample the means of each class in each feature space.
  vector< vector<VectorXf> > means(num_classes_); //means[i][j] is the mean for the ith class, jth feature space.
  for(size_t i=0; i<num_classes_; ++i) {
    means[i] = vector<VectorXf>(num_feature_spaces_);
    for(size_t j=0; j<num_feature_spaces_; ++j) {
      means[i][j] = VectorXf::Random(num_dims_) * 100;
    }
  }
  
  // -- Generate the objects.
  vector<Object*> objs(0);
  objs.reserve(num_objs_ + num_bg_);
  size_t ctr = 0;
  while(ctr < num_objs_) {
    Object* obj = new Object();
    obj->descriptors_ = vector<descriptor>(num_feature_spaces_);
    int lbl = rand() % num_classes_;
    if(labeled_) { 
      obj->label_ = lbl;
      // Randomly flip the labels of some objects.
/*       if((double)rand() / (double)RAND_MAX >= 0.98) */
/* 	obj->label_ = rand() % num_classes_; */
    }
    else
      obj->label_ = -2;
    for(size_t j=0; j<num_feature_spaces_; ++j) {
      if((double)rand() / (double)RAND_MAX <= feature_probability_) {
	obj->descriptors_[j].vector = sampleFromMultidimGaussian(means[lbl][j], variance_);
	obj->descriptors_[j].length_squared = obj->descriptors_[j].vector->dot(*obj->descriptors_[j].vector);
      }
      else {
	obj->descriptors_[j].vector = NULL;
	obj->descriptors_[j].length_squared = 0;
      }
    }
    objs.push_back(obj);
    ctr++;
  }
   
  // -- Generate random background data.
  while(ctr < num_objs_ + num_bg_) { 
    Object* obj = new Object();
    obj->descriptors_ = vector<descriptor>(num_feature_spaces_);
    if(labeled_) 
      obj->label_ = -1;
    else
      obj->label_ = -2;
    for(size_t j=0; j<num_feature_spaces_; ++j) {
      if((double)rand() / (double)RAND_MAX <= feature_probability_) {
	obj->descriptors_[j].vector = new VectorXf;
	*obj->descriptors_[j].vector = VectorXf::Random(num_dims_)*100;
	obj->descriptors_[j].length_squared = obj->descriptors_[j].vector->dot(*obj->descriptors_[j].vector);
      }
      else {
	obj->descriptors_[j].vector = NULL;
	obj->descriptors_[j].length_squared = 0;
      }
    }
    objs.push_back(obj);
    ctr++;
  }

  // -- Make a new dataset from this and return it.
  MultiBoosterDataset* mbd = new MultiBoosterDataset(classes, feature_spaces);
  mbd->setObjs(objs);
  return mbd;
}
