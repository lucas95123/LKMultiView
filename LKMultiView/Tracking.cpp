#include "Tracking.h"
void extract_features(
	vector<string>& image_names,
	vector<vector<KeyPoint>>& vecKeyPointsAll,
	vector<Mat>& vecDescriptorAll,
	vector<vector<Vec3b>>& vecColorsAll
)
{
	vecKeyPointsAll.clear();
	vecDescriptorAll.clear();
	Mat image;

	//create SIFT feature detector
	Ptr<Feature2D> sift = xfeatures2d::SIFT::create();
	for (auto it = image_names.begin(); it != image_names.end(); ++it)
	{
		image = imread(*it);
		if (image.empty()) continue;

		cout << "Extracing features: " << *it << endl;

		vector<KeyPoint> key_points;
		Mat descriptor;
		//detect SIFT features in image and compute the descriptor of the feature
		sift->detectAndCompute(image, noArray(), key_points, descriptor);

		//if the number of SIFT features is too small
		if (key_points.size() <= 10) continue;

		//save the keypoints of this image
		vecKeyPointsAll.push_back(key_points);

		//save the keypoints of the descriptor of the image
		vecDescriptorAll.push_back(descriptor);

		//save the color of the key points of the image
		vector<Vec3b> colors(key_points.size());
		for (int i = 0; i < key_points.size(); ++i)
		{
			Point2f& p = key_points[i].pt;
			colors[i] = image.at<Vec3b>(p.y, p.x);
		}
		vecColorsAll.push_back(colors);
	}
}

void match_features(Mat& descL, Mat& descR, vector<DMatch>& matches)
{
	static const int kFlannMaxDistScale = 3;
	static const double kFlannMaxDistThreshold = 0.04;
	// FlannBasedMatcher with KD-Trees needs CV_32
	descL.convertTo(descL, CV_32F);
	descR.convertTo(descR, CV_32F);

	// KD-Tree param: # of parallel kd-trees
	static const int kFlannNumTrees = 4;
	FlannBasedMatcher matcher(new flann::KDTreeIndexParams(kFlannNumTrees));
	vector<DMatch> flannMatches;
	matcher.match(descL, descR, flannMatches);

	double maxDist = 0;
	double minDist = numeric_limits<float>::max();
	for (int i = 0; i < flannMatches.size(); ++i) {
		double dist = flannMatches[i].distance;
		maxDist = max(maxDist, dist);
		minDist = min(minDist, dist);
	}

	for (int i = 0; i < flannMatches.size(); ++i) {
		double distThresh = kFlannMaxDistScale * minDist;
		if (flannMatches[i].distance <= max(distThresh, kFlannMaxDistThreshold)) {
			matches.push_back(flannMatches[i]);
		}
	}
}

void match_features(vector<Mat>& vecDescriptorAll, vector<vector<DMatch>>& vecMatchesAll)
{
	vecMatchesAll.clear();
	//match features for every adjacent image
	for (int i = 0; i < vecDescriptorAll.size() - 1; ++i)
	{
		cout << "Matching images " << i << " - " << i + 1 << endl;
		vector<DMatch> matches;
		match_features(vecDescriptorAll[i], vecDescriptorAll[i + 1], matches);
		vecMatchesAll.push_back(matches);
	}
}

bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T, Mat& mask)
{
	Mat F = findFundamentalMat(p1, p2, CV_FM_8POINT);

	//calculate the essential matrix
	Mat E = findEssentialMat(p1, p2, K, RANSAC, 0.9989999999, 1.0, mask);
	if (E.empty()) return false;

	double feasible_count = countNonZero(mask);
	cout << (int)feasible_count << " -in- " << p1.size() << endl;
	//if there are too much outliers return false
	if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)
		return false;

	//recover the pose from essential matrix
	int pass_count = recoverPose(E, p1, p2, K, R, T, mask);

	//if again there are too much outliers return false
	if (((double)pass_count) / feasible_count < 0.7)
		return false;

	return true;
}

void get_matched_points(
	vector<KeyPoint>& p1,
	vector<KeyPoint>& p2,
	vector<DMatch> matches,
	vector<Point2f>& out_p1,
	vector<Point2f>& out_p2
)
{
	out_p1.clear();
	out_p2.clear();
	for (int i = 0; i < matches.size(); ++i)
	{
		out_p1.push_back(p1[matches[i].queryIdx].pt);
		out_p2.push_back(p2[matches[i].trainIdx].pt);
	}
}

void get_matched_colors(
	vector<Vec3b>& c1,
	vector<Vec3b>& c2,
	vector<DMatch> matches,
	vector<Vec3b>& out_c1,
	vector<Vec3b>& out_c2
)
{
	out_c1.clear();
	out_c2.clear();
	for (int i = 0; i < matches.size(); ++i)
	{
		out_c1.push_back(c1[matches[i].queryIdx]);
		out_c2.push_back(c2[matches[i].trainIdx]);
	}
}

void reconstruct(Mat& K, Mat& R1, Mat& T1, Mat& R2, Mat& T2, vector<Point2f>& p1, vector<Point2f>& p2, vector<Point3f>& structure)
{
	//the projection matrix of the cameras
	Mat proj1(3, 4, CV_32FC1);
	Mat proj2(3, 4, CV_32FC1);

	R1.convertTo(proj1(Range(0, 3), Range(0, 3)), CV_32FC1);
	T1.convertTo(proj1.col(3), CV_32FC1);

	R2.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
	T2.convertTo(proj2.col(3), CV_32FC1);

	Mat fK;
	K.convertTo(fK, CV_32FC1);
	proj1 = fK*proj1;
	proj2 = fK*proj2;

	//the output homogeneous points
	Mat s;
	triangulatePoints(proj1, proj2, p1, p2, s);

	structure.clear();
	structure.reserve(s.cols);
	for (int i = 0; i < s.cols; ++i)
	{
		Mat_<float> col = s.col(i);
		//convert from homogeneous coordinates
		col /= col(3);
		structure.push_back(Point3f(col(0), col(1), col(2)));
	}
}

void maskout_points(vector<Point2f>& p1, Mat& mask)
{
	vector<Point2f> p1_copy = p1;
	p1.clear();

	for (int i = 0; i < mask.rows; ++i)
	{
		if (mask.at<uchar>(i) > 0)
			p1.push_back(p1_copy[i]);
	}
}

void maskout_colors(vector<Vec3b>& p1, Mat& mask)
{
	vector<Vec3b> p1_copy = p1;
	p1.clear();

	for (int i = 0; i < mask.rows; ++i)
	{
		if (mask.at<uchar>(i) > 0)
			p1.push_back(p1_copy[i]);
	}
}

void save_structure(string file_name, vector<Mat>& rotations, vector<Mat>& motions, vector<Point3f>& structure, vector<Vec3b>& colors)
{
	int n = (int)rotations.size();

	FileStorage fs(file_name, FileStorage::WRITE);
	fs << "Camera Count" << n;
	fs << "Point Count" << (int)structure.size();

	fs << "Rotations" << "[";
	for (size_t i = 0; i < n; ++i)
	{
		fs << rotations[i];
	}
	fs << "]";

	fs << "Motions" << "[";
	for (size_t i = 0; i < n; ++i)
	{
		fs << motions[i];
	}
	fs << "]";

	fs << "Points" << "[";
	for (size_t i = 0; i < structure.size(); ++i)
	{
		fs << structure[i];
	}
	fs << "]";

	fs << "Colors" << "[";
	for (size_t i = 0; i < colors.size(); ++i)
	{
		fs << colors[i];
	}
	fs << "]";

	fs.release();
}

void get_objpoints_and_imgpoints(
	vector<DMatch>& matches,
	vector<int>& struct_indices,
	vector<Point3f>& structure,
	vector<KeyPoint>& key_points,
	vector<Point3f>& object_points,
	vector<Point2f>& image_points)
{
	object_points.clear();
	image_points.clear();

	for (int i = 0; i < matches.size(); ++i)
	{
		int query_idx = matches[i].queryIdx;
		int train_idx = matches[i].trainIdx;

		int struct_idx = struct_indices[query_idx];
		if (struct_idx < 0) continue;

		object_points.push_back(structure[struct_idx]);
		image_points.push_back(key_points[train_idx].pt);
	}
}

void fusion_structure(
	vector<DMatch>& matches,
	vector<int>& struct_indices,
	vector<int>& next_struct_indices,
	vector<Point3f>& structure,
	vector<Point3f>& next_structure,
	vector<Vec3b>& colors,
	vector<Vec3b>& next_colors
)
{
	for (int i = 0; i < matches.size(); ++i)
	{
		int query_idx = matches[i].queryIdx;
		int train_idx = matches[i].trainIdx;

		int struct_idx = struct_indices[query_idx];
		if (struct_idx >= 0)
		{
			next_struct_indices[train_idx] = struct_idx;
			continue;
		}

		structure.push_back(next_structure[i]);
		colors.push_back(next_colors[i]);
		struct_indices[query_idx] = next_struct_indices[train_idx] = structure.size() - 1;
	}
}

void init_structure(
	Mat K,
	vector<vector<KeyPoint>>& vecKeyPointsAll,
	vector<vector<Vec3b>>& vecColorsAll,
	vector<vector<DMatch>>& vecMatchesAll,
	vector<Point3f>& structure,
	vector<vector<int>>& correspond_struct_idx,
	vector<Vec3b>& colors,
	vector<Mat>& rotations,
	vector<Mat>& motions
)
{
	//the matched points in the first and second image
	vector<Point2f> p1, p2;
	//the color of the matched points
	vector<Vec3b> c2;
	Mat R, T;	//the rotation and translation matrix between the first two images
	Mat mask;	//mask of keypoints that is weak
	get_matched_points(vecKeyPointsAll[0], vecKeyPointsAll[1], vecMatchesAll[0], p1, p2);
	get_matched_colors(vecColorsAll[0], vecColorsAll[1], vecMatchesAll[0], colors, c2);
	find_transform(K, p1, p2, R, T, mask);

	//mask the outliers
	maskout_points(p1, mask);
	maskout_points(p2, mask);
	maskout_colors(colors, mask);

	Mat R0 = Mat::eye(3, 3, CV_64FC1);
	Mat T0 = Mat::zeros(3, 1, CV_64FC1);
	reconstruct(K, R0, T0, R, T, p1, p2, structure);

	//initialize the rotation vector and translation vector
	rotations = { R0, R };
	motions = { T0, T };

	//
	correspond_struct_idx.clear();
	correspond_struct_idx.resize(vecKeyPointsAll.size());
	for (int i = 0; i < vecKeyPointsAll.size(); ++i)
	{
		correspond_struct_idx[i].resize(vecKeyPointsAll[i].size(), -1);
	}

	//
	int idx = 0;
	vector<DMatch>& matches = vecMatchesAll[0];
	for (int i = 0; i < matches.size(); ++i)
	{
		if (mask.at<uchar>(i) == 0)
			continue;

		correspond_struct_idx[0][matches[i].queryIdx] = idx;
		correspond_struct_idx[1][matches[i].trainIdx] = idx;
		++idx;
	}
}

void trackCamera(
	vector<string> &vecImgNames,
	Mat& K, 
	vector<Mat> &rotations, 
	vector<Mat> &translations, 
	bool saveStructure,
	string saveDir)
{
	//vector storing keypoints of each image
	vector<vector<KeyPoint>> vecKeyPointsAll;
	//vector storing descriptor of keypoints in each image
	vector<Mat> vecDescriptorAll;
	//vector storing color of keypoints in each image
	vector<vector<Vec3b>> vecColorsAll;
	//vector storing matches
	vector<vector<DMatch>> vecMatchesAll;
	//extracting SIFT feature from all images
	extract_features(vecImgNames, vecKeyPointsAll, vecDescriptorAll, vecColorsAll);
	//match the feature between images
	match_features(vecDescriptorAll, vecMatchesAll);

	vector<Point3f> structure;
	vector<vector<int>> correspond_struct_idx; //保存第i副图像中第j个特征点对应的structure中点的索引
	vector<Vec3b> colors;

	//get the initial rotation and translation between the first two images using essential matrix
	init_structure(
		K,
		vecKeyPointsAll,
		vecColorsAll,
		vecMatchesAll,
		structure,
		correspond_struct_idx,
		colors,
		rotations,
		translations
	);

	//calcuate camera pose for other images
	for (int i = 1; i < vecMatchesAll.size(); ++i)
	{
		vector<Point3f> object_points;
		vector<Point2f> image_points;
		Mat r, R, T;
		//Mat mask;

		//get the object points and image points
		get_objpoints_and_imgpoints(
			vecMatchesAll[i],
			correspond_struct_idx[i],
			structure,
			vecKeyPointsAll[i + 1],
			object_points,
			image_points
		);

		//solve the rotation and translation
		solvePnPRansac(object_points, image_points, K, noArray(), r, T);
		//from rotation matrix to rotation vectors
		Rodrigues(r, R);
		//push the rotation and translation back
		rotations.push_back(R);
		translations.push_back(T);

		vector<Point2f> p1, p2;
		vector<Vec3b> c1, c2;
		get_matched_points(vecKeyPointsAll[i], vecKeyPointsAll[i + 1], vecMatchesAll[i], p1, p2);
		get_matched_colors(vecColorsAll[i], vecColorsAll[i + 1], vecMatchesAll[i], c1, c2);

		//by current rotation and translation reconstruct the point cloud of the current two images
		vector<Point3f> next_structure;
		reconstruct(K, rotations[i], translations[i], R, T, p1, p2, next_structure);

		//fusion the current structure with the previous structure
		fusion_structure(
			vecMatchesAll[i],
			correspond_struct_idx[i],
			correspond_struct_idx[i + 1],
			structure,
			next_structure,
			colors,
			c1
		);
	}

	//save the camera and structures
	if(saveStructure)
		save_structure(saveDir+".\\structure.yml", rotations, translations, structure, colors);
}
