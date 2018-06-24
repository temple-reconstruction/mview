#include "mview.h"
#include "Eigen.h"

static Pointcloud align_to_existing(const Pointcloud& reference, const Pointcloud& to_align);
static void merge_pointcloud_into(Pointcloud& target, Pointcloud input);

auto align(std::vector<Pointcloud> pointclouds) -> Pointcloud {
	Pointcloud output;
	for(const auto& pointcloud : pointclouds) {
		Pointcloud aligned = align_to_existing(output, pointcloud);
		merge_pointcloud_into(output, aligned);
	}
}

Pointcloud align_to_existing(const Pointcloud& reference, Pointcloud& to_align) {
	// FIXME: ICP or another alignment optimization
	int n1=reference.points.size();
	int n2=to_align.points.size();
	int n3=0;
	int maxIter = 10;
	double dist=0;
	std::vector<int> nn_idx(n2, 0); //nearest neighbour
	std::vector<double> nn_distance(n2, 0.0);
	std::vector<double> nn_median(n2, 0.0); //to compute median value
	for (int it=0;it<maxIter;it++){

		//computer nearest neighour for each point in to_align
		for (int i=0;i<n2;i++){
			double min_dist=2018.0;
			for (int j=0;j<n1;j++){
				dist=((reference.points[j]-to_align.points[i]).squaredNorm());
				if (dist<min_dist){
					nn_idx[i]=j;
					min_dist=dist;
				}
			}
			nn_distance[i]=min_dist;
			nn_median[i]=min_dist;
		}

		//discarding outlier pairs with dist > k * median
		double k=1.0;
		double median=0.0;
		std::sort(nn_median.begin(), nn_median.end());
		if (n2%2==1){
			median=nn_median[(n2+1)/2];
		}
		else{
			median=(nn_median[n2/2-1]+nn_median[n2/2])/2;
		}
		for (int i=0;i<n2;i++){
			if (nn_distance[i]>k*median){
				nn_idx[i]=-1;
				nn_distance[i]=-1.0;
			}
			else{
				n3++;
			}
		}

		//Find the R,T that minimize E=sum (Rp+T-q)^2
		//1. write out inliers
		Eigen::MatrixXf P(3,n3); //from to_align
		Eigen::MatrixXf Q(3,n3); //from reference
		int count=0;
		for (int i=0;i<n2;i++){
			if (nn_idx[i]>=0){
				P(0,count)=to_align.points[i][0];
				P(1,count)=to_align.points[i][1];
				P(2,count)=to_align.points[i][2];
				Q(0,count)=reference.points[nn_idx[i]][0];
				Q(1,count)=reference.points[nn_idx[i]][1];
				Q(2,count)=reference.points[nn_idx[i]][2];
				count++;
			}
		}

		//2. compute mean of inliers and centerize
		Eigen::Vector3f mean_ref(0.0, 0.0, 0.0);
		Eigen::Vector3f mean_source(0.0, 0.0, 0.0);
		for (int i=0;i<n3;i++){
			mean_source[0]=mean_source[0]+P(0,i)/((double)n3);
			mean_source[1]=mean_source[1]+P(1,i)/((double)n3);
			mean_source[2]=mean_source[2]+P(2,i)/((double)n3);
			mean_ref[0]=mean_ref[0]+Q(0,i)/((double)n3);
			mean_ref[1]=mean_ref[1]+Q(1,i)/((double)n3);
			mean_ref[2]=mean_ref[2]+Q(2,i)/((double)n3);
		}
		for (int i=0;i<n3;i++){
			P(0,i)=P(0,i)-mean_source[0];
			P(1,i)=P(1,i)-mean_source[1];
			P(2,i)=P(2,i)-mean_source[2];
			Q(0,i)=Q(0,i)-mean_ref[0];
			Q(1,i)=Q(1,i)-mean_ref[1];
			Q(2,i)=Q(2,i)-mean_ref[2];
		}

		//3. SVD like Procrustes, compute R, T
		MatrixXf CovarianceMat=Q*(P.transpose());

		JacobiSVD<MatrixXf> svd( CovarianceMat, ComputeFullU | ComputeFullV);

		Matrix3f R=svd.matrixU() * (svd.matrixV().transpose());
		Vector3f translation=mean_ref-mean_source;
		Vector3f T=R*translation - R * mean_ref + mean_ref;//not sure here

		//update to_algin
		Vector3f new_position(0.0,0.0,0.0);
		for (int i=0;i<n2;i++){
			new_position=R*to_align.points[i]+T;
			to_align.points[i][0]=new_position[0];
			to_align.points[i][1]=new_position[1];
			to_align.points[i][2]=new_position[2];
		}
	}
	return to_align;
}

void merge_pointcloud_into(Pointcloud& target, Pointcloud input) {
	target.points.insert(target.points.end(), input.points.begin(), input.points.end());
}
