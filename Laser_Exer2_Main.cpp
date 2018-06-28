#include <direct.h>
#include "Scan_Point.h"
#include "TXTReader.h"
#include "TXTWriter.h"
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

// function deletes zeros from vector of pointers to Scan_Points
void Delete_zeros_1D(vector<Scan_Point*> &points) {
	int i = 0;
	// while on the position i there is element with x = y = z = 0, delete element i
	while (i < points.size())
	{ 
		if (points[i]->x == 0 && points[i]->y == 0 && points[i]->z == 0)
			points.erase(points.begin() + i);
		else
			i++;
	}
}

// generates rotation matrix R defined by angle phi around axis axis_number
Mat Rot_mat(float phi, short axis_number)
{
	Mat R;
	switch (axis_number) 
	{
		case 0: R = (Mat_<float>(4, 4) << 1,        0,         0, 0,
										  0, cos(phi), -sin(phi), 0,
										  0, sin(phi), cos(phi),  0,
										  0,        0,        0,  1);
			break;
		case 1: R = (Mat_<float>(4, 4) << cos(phi), 0, sin(phi), 0,
										         0, 1,        0, 0,
										 -sin(phi), 0, cos(phi), 0,
												 0, 0,        0, 1); 
			break;
		case 2: R = (Mat_<float>(4, 4) << cos(phi), -sin(phi), 0, 0,
										  sin(phi),  cos(phi), 0, 0,
												 0,         0, 1, 0,
											     0,         0, 0, 1); 
			break;
	}
	return R;
}

// function finds centroid of a pointcloud given as vector of pointers to Scan_Points
Scan_Point Point_cloud_centroid(vector<Scan_Point*> p_points) {
	Scan_Point centroid = Scan_Point(0, 0, 0, 0, 0, 0, 0);
	for (Scan_Point* p_p : p_points) {
		centroid.x += p_p->x;
		centroid.y += p_p->y;
		centroid.z += p_p->z;
	}
	centroid.x = centroid.x / p_points.size();
	centroid.y = centroid.y / p_points.size();
	centroid.z = centroid.z / p_points.size();
	return centroid;
}

// function performs intensity stretch for vector of pointers to Scan_Points avoiding zero-points
void Stretch_intensity_2D(const vector<vector<Scan_Point*>> &p_points)
{
	float max_inten = 0.0, min_inten = 255.0;

	// calculate minimal and maximal intensities
	for (const vector<Scan_Point*> vec_p_points : p_points) {
		for (const Scan_Point* p_p : vec_p_points)
		{
			if (p_p->I > max_inten)
				max_inten = p_p->I;
			if (p_p->I < min_inten && (p_p->x != 0.0 || p_p->y != 0.0 || p_p->z != 0.0))
			min_inten = p_p->I;
		}
	}
	cout << "Initial values of intensity have minimum value " << min_inten << " and maximum value " << max_inten << endl;

	// stretch intensities
	for (const vector<Scan_Point*> vec_p_points : p_points) {
		for (Scan_Point* p_p : vec_p_points)
		{
			if (p_p->x != 0.0 || p_p->y != 0.0 || p_p->z != 0.0) p_p->I = round(255 * (p_p->I - min_inten) / (max_inten - min_inten));
		}
	}
}

// crop projected image which is initially built on the white canvas
void Crop_Image(Mat& img) {
	int row_top, row_bot, col_lef, col_rig;
	for (row_top = 0; row_top < img.rows; row_top++) { // for every row from upper side
		if (sum(sum(img.row(row_top)))[0] != 255 * img.cols * img.channels()) break; // if not white found, break
	}
	for (row_bot = img.rows - 1; row_bot > -1; row_bot--) { // for every row from bottom side
		if (sum(sum(img.row(row_bot)))[0] != 255 * img.cols * img.channels()) break; // if not white found, break
	}
	for (col_lef = 0; col_lef < img.cols; col_lef++) { // for every col from left side
		if (sum(sum(img.col(col_lef)))[0] != 255 * img.rows * img.channels()) break; // if not white found, break
	}
	for (col_rig = img.cols - 1; col_rig > -1; col_rig--) { // for every col from right side
		if (sum(sum(img.col(col_rig)))[0] != 255 * img.rows * img.channels()) break; // if not white found, break
	}
	img = img.rowRange(row_top, row_bot + 1).colRange(col_lef, col_rig + 1);
}

// transforms vector of scanned points into image using projective geometry
Mat Vector_of_Scanpoints_to_Projected_Image(vector<Scan_Point*> &p_points, bool grayscale, int img_size) {
	
	int row = 0, col = 0;
	int prin_dist = img_size;

	Vec3f pixel;
	Mat result;
	Mat white_1c = 255 * Mat::ones(img_size, img_size, CV_32FC1);

	// initialisation of resulting matrix
	if (grayscale) result = white_1c;
	else {
		result = Mat::zeros(img_size, img_size, CV_32FC3);
		vector<Mat> white_3c;
		for (int i = 0; i < 3; i++) white_3c.push_back(white_1c);
		merge(white_3c, result);
	}

	// set up calibration and rotation-translation matrices
	Mat K = (Mat_<float>(3, 3) << prin_dist, 0, img_size / 2,
		0, prin_dist, img_size / 2,
		0, 0, 1);
	Mat M34 = (Mat_<float>(3, 4) << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0);
	Mat XYZW = Mat::ones(4, 1, CV_32FC1);
	Mat uvq = Mat::ones(3, 1, CV_32FC1);
	Mat M1, M2, M3;
	Scan_Point centroid = Point_cloud_centroid(p_points);
	float azimuth = atan2(centroid.x, centroid.y);
	float height = atan2(centroid.z, sqrt(pow(centroid.x, 2) + pow(centroid.y, 2)));

	// for every point of input
	for (const Scan_Point* p_p : p_points) {

		// calculate row and col - coordinates on the image
		XYZW.at<float>(0) = p_p->x;
		XYZW.at<float>(1) = p_p->y;
		XYZW.at<float>(2) = p_p->z;
		M1 = Rot_mat(azimuth, 2) * XYZW;
		M2 = Rot_mat(M_PI / 2 - height, 0) * M1;
		M3 = M34 * M2;
		uvq = K * M3;
		col = (int)(uvq.at<float>(0) / uvq.at<float>(2));
		row = (int)(uvq.at<float>(1) / uvq.at<float>(2));

		// assign value to the corresponding
		if (grayscale) {
			result.at<float>(row, col) = p_p->I; // * 255
		}
		else {
			pixel.val[0] = p_p->B;
			pixel.val[1] = p_p->G;
			pixel.val[2] = p_p->R;
			result.at<Vec3f>(row,col) = pixel; // j, i because i is the col number and j is row number
		}
	}
	Crop_Image(result); // crop not-used area
	return result;
}

// function transforms vector of vectors of scanpoints into rectangular image
Mat Matrix_of_Scanpoints_to_Rectangular_Image(vector<vector<Scan_Point*>> &input, bool grayscale) {
	Mat result, result_flipped;

	if (grayscale) {
		result = 255 * Mat::ones(input[0].size(), input.size(), CV_32FC1);
		for (int i = 0; i < input.size(); i++) { // i is current col number
			for (int j = 0; j < input[i].size(); j++) { // j is current row number
				result.at<float>(j, i) = input[i][j]->I; // j, i because i is the col number and j is row number          //*255
			}
		}
	}
	else {
		result = Mat::ones(input[0].size(), input.size(), CV_32FC3);
		Vec3f pixel;
		for (int i = 0; i < input.size(); i++) { // i is current col number
			for (int j = 0; j < input[i].size(); j++) { // j is current row number
				pixel.val[0] = input[i][j]->B;
				pixel.val[1] = input[i][j]->G;
				pixel.val[2] = input[i][j]->R;
				result.at<Vec3f>(j, i) = pixel; // j, i because i is the col number and j is row number
			}
		}
	}
	result_flipped = result.clone();
	flip(result, result_flipped, 0); // flip, because enumeration inside a column of an image goes from up to down, of an vec<vec<scanpoint>> - from down to up
	return result_flipped;
}

// function transforms vector of vector of scanpoints into vector of scanpoints
vector<Scan_Point*> Matrix_2D_to_1D(vector<vector<Scan_Point*>> &input) {
	vector<Scan_Point*> result;
	for (int i = 0; i < input.size(); i++) {
		for (int j = 0; j < input[i].size(); j++) {
			result.push_back(input[i][j]);
		}
	}
	return result;
}

// function creates a 2D kernel based on gaussian partial derivative in the x-direction
Mat createFstDevKernel(int kSize) {
	double sigma = double((kSize + 1) / 3) / 2;
	Mat gaussKernel = getGaussianKernel(kSize, sigma, CV_32FC1); // 1D matrix (kSize x 1) of gaussian coefficients 
	gaussKernel = gaussKernel * gaussKernel.t(); // 2D Gauss kernel

	// first derivative of 2D Gauss kernel
	for (int i = 0; i < gaussKernel.size().height; i++)
		for (int j = 0; j < gaussKernel.size().width; j++) {
			gaussKernel.at<float>(i, j) = gaussKernel.at<float>(i, j) * ((kSize - 1) / 2 - j) / pow(sigma, 2);
		}
	return gaussKernel;
}

// non-maxima suppression (if any of the pixel at the 4-neighborhood is greater than current pixel, set current pixel to zero)
Mat nonMaxSuppression(Mat& img) {

	Mat out = img.clone();

	for (int x = 1; x < out.cols - 1; x++) {
		for (int y = 1; y < out.rows - 1; y++) {
			if (img.at<float>(y - 1, x) >= img.at<float>(y, x)) {
				out.at<float>(y, x) = 0;
				continue;
			}
			if (img.at<float>(y, x - 1) >= img.at<float>(y, x)) {
				out.at<float>(y, x) = 0;
				continue;
			}
			if (img.at<float>(y, x + 1) >= img.at<float>(y, x)) {
				out.at<float>(y, x) = 0;
				continue;
			}
			if (img.at<float>(y + 1, x) >= img.at<float>(y, x)) {
				out.at<float>(y, x) = 0;
				continue;
			}
		}
		out.at<float>(0, x) = 0; // avoid finding interest points on the upper border of the image
		out.at<float>(out.rows - 1, x) = 0; // lower border
	}

	for (int y = 0; y < out.rows; y++) {
		out.at<float>(y, 0) = 0; // left border
		out.at<float>(y, out.cols - 1) = 0; // and right border
	}
	return out;
}

// function performs interest point detection on the img using hyperparameters kSize, thresh_wei, thresh_iso and saves it to value "points"
void getInterestPoints(Mat& img, int kSize, vector<KeyPoint>& points, float thresh_wei, float thresh_iso) {
	int border_width = kSize / 2;
	Mat img_ext = Mat::zeros(img.rows + 2 * border_width, img.cols + 2 * border_width, CV_32FC1);
	copyMakeBorder(img, img_ext, border_width, border_width, border_width, border_width, BORDER_REPLICATE);
	Mat grad_x = img_ext.clone(); // grad_x and grad_y are clones of img_ext : image with addition of boarders
	Mat grad_y = img_ext.clone();
	Mat T = Mat::zeros(2, 2, CV_32FC1);

	// create matrices for gradient values
	Mat grad_x_sq_win = Mat::zeros(kSize, kSize, CV_32FC1);
	Mat grad_y_sq_win = Mat::zeros(kSize, kSize, CV_32FC1);
	Mat grad_xy_win = Mat::zeros(kSize, kSize, CV_32FC1);
	Mat grad_x_sq;
	Mat grad_y_sq;
	Mat grad_xy;

	// create matrices of gradients
	Mat Grad_Kernel = createFstDevKernel(kSize); // Grad_Kernel is first derivative of gaussian in x direction								
	filter2D(grad_x, grad_x, CV_32FC1, Grad_Kernel); // convolution of grad_x by 1-deriv of gaussian in x direction, now grad_x is gradient of orig in x
	filter2D(grad_y, grad_y, CV_32FC1, Grad_Kernel.t()); // convolution of grad_y by 1-deriv of gaussian in y direction, now grad_y is gradient of orig in y

	Mat tr = Mat::zeros(img.rows, img.cols, CV_32FC1);
	Mat det = Mat::zeros(img.rows, img.cols, CV_32FC1);
	Mat wei = Mat::zeros(img.rows, img.cols, CV_32FC1);
	Mat iso = Mat::zeros(img.rows, img.cols, CV_32FC1);

	pow(grad_x, 2, grad_x_sq); // gradient values inside window are squared and placed on the diagonal of tensor
	pow(grad_y, 2, grad_y_sq);
	multiply(grad_x, grad_y, grad_xy);

	// calculate matrices of traces, determinants, weights and isotropies
	for (int i = 0; i < img.rows; i++) // for each element     
		for (int j = 0; j < img.cols; j++) {

			grad_x_sq_win = grad_x_sq.rowRange(i, i + kSize).colRange(j, j + kSize); // grad_x_win is window of kSize from grad_x
			grad_y_sq_win = grad_y_sq.rowRange(i, i + kSize).colRange(j, j + kSize); // grad_y_win is window of kSize from grad_y
			grad_xy_win = grad_xy.rowRange(i, i + kSize).colRange(j, j + kSize);

			// Tensor for current pixel
			T.at<float>(0, 0) = sum(grad_x_sq_win)[0];
			T.at<float>(0, 1) = sum(grad_xy_win)[0];
			T.at<float>(1, 0) = sum(grad_xy_win)[0];
			T.at<float>(1, 1) = sum(grad_y_sq_win)[0];

			// Trace of the tensor for current pixel
			tr.at<float>(i, j) = trace(T)[0];

			// Determinant of the tensor for current pixel
			det.at<float>(i, j) = determinant(T);

			// Weight and isotropy are calculated from determinant and trace of tensor for current pixel
			if (tr.at<float>(i, j) != 0) {
				wei.at<float>(i, j) = det.at<float>(i, j) / tr.at<float>(i, j);
				iso.at<float>(i, j) = 4 * det.at<float>(i, j) / pow(tr.at<float>(i, j), 2);
			}
			else {
				wei.at<float>(i, j) = 0;
				iso.at<float>(i, j) = 0;
			}

		}

	// non-max suppression leaves local maximums only
	wei = nonMaxSuppression(wei);
	iso = nonMaxSuppression(iso);

	// thresholding
	float aver_wei = sum(wei)[0] / (wei.cols * wei.rows); // average weight
	threshold(wei, wei, thresh_wei * aver_wei, 1, THRESH_BINARY); // weight min depends on thresh_wei
	threshold(iso, iso, thresh_iso, 1, THRESH_BINARY); // isotropy min depends only on threshold of isotropy
	threshold(wei + iso, iso, 1, 1, THRESH_BINARY); // binary result is stored in iso matrix 

	// based on iso matrix create KeyPoint and push it back
	KeyPoint kp;
	for (int i = 0; i < iso.rows; i++)
		for (int j = 0; j < iso.cols; j++) {
			if (iso.at<float>(i, j) > 0) {
				kp.pt.x = j;
				kp.pt.y = i;
				kp.size = kSize; 
				points.push_back(kp);
			}
		}
}

// function detects keypoints on img, draws it on image and stores ot in a file
vector<KeyPoint> detectAndSaveInterestPoints(Mat& img, int kSize, float thresh_wei, float thresh_iso, bool grayscale, string filepath) {
	vector<KeyPoint> points = {};
	getInterestPoints(img, kSize, points, thresh_wei, thresh_iso);
	cvtColor(img, img, COLOR_GRAY2BGR); // because we want keypoints to be marked by red color
	drawKeypoints(img, points, img, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_OVER_OUTIMG + DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	imwrite((filepath + string(".png")).c_str(), img);
	return points;
}

// function converts vector of keypoints to vector of scanpoints and stores it to a file
void convertKeypointsToScanPoints(vector<KeyPoint>& kpts_vec, vector<vector<Scan_Point*>>& mat_p_scp, string filepath) {
	// now points is vector of Keypoints, which are defined by their pixel coordinates
	vector<Scan_Point*> p_keyscp;

	// to get vector of ScanPoints from vector of keypoints we need
	for (int i = 0; i < kpts_vec.size(); i++) {  // for every member of keypoints vector
		p_keyscp.push_back(mat_p_scp[kpts_vec[i].pt.x][mat_p_scp[0].size() - 1 - kpts_vec[i].pt.y]); // pushback the corresponding member of mat_p_scp to vector of pointers to ScanPoints
	}

	// save p_keypts to a ptx file
	TXTWriter::WritePointPointersPTX(filepath, p_keyscp);
}

// function returns an image histogram
Mat createAndSaveHistogram(Mat& img) {
	// Establish the number of bins
	int histSize = 256;

	// Set the ranges ( for B,G,R) )
	float range[] = { 0, 256 };
	const float* histRange = { range };
	bool uniform = true; bool accumulate = false;
	Mat histogram;

	// Compute the histograms:
	calcHist(&img, 1, 0, Mat(), histogram, 1, &histSize, &histRange, uniform, accumulate);
	 
	int hist_w = 1024; int hist_h = 400;
	int bin_w = cvRound((double)hist_w / histSize);
	Mat histImage = 255 * Mat::ones(hist_h+1, hist_w, CV_8UC1);

	// Normalize the result to [ 0, histImage.rows ]
	normalize(histogram, histogram, 0, histImage.rows, NORM_MINMAX, -1, Mat());

	for (int i = 1; i < histSize; i++)
	{
		line(histImage, Point(bin_w*(i-1), hist_h - cvRound(histogram.at<float>(i-1))), Point(bin_w*i, hist_h - cvRound(histogram.at<float>(i))), 0, 1, 8, 0);
	}

	return histImage;
}

// function assigns intensity 0 to pixels with coordinates (0,0,0)
void assignBlackColorToWeirdPixels(vector<vector<Scan_Point*>> &input) {
	for (int i = 0; i < input.size(); i++) {
		for (int j = 0; j < input[i].size(); j++) {
			if (input[i][j]->x == 0 && input[i][j]->y == 0 && input[i][j]->z == 0) {
				input[i][j]->I = 0.0;
			}
		}
	}
}

int main()
{
	// Processing TU_outside dataset
	int kSize = 5;
	float thresh_wei = 1.5; 
	float thresh_iso;
	string folder_name = "C:\\Users\\user\\GoogleDisk\\Study\\Laser\\Laser_Exers\\Laser_Exer2\\Assignment2\\TU-Main-Building\\";
	string file_name;

	int users_choise;
	cout << "Please make a choise of processed point cloud:" << endl << " 0 - outside of TU (TU_outside.ptx)" << endl;
	cout << " 1 - first image of TU interior (a87aaa504.ptx)" << endl << " 2 - second image of TU interior (a87aaa505.ptx)" << endl;
	cin >> users_choise;

	if (users_choise == 0) {
		thresh_iso = 0.99; //0.75 for interior, 0.99 for exterior dataset
		file_name = "TU_outside";

		// read initial pointcloud
		cout << "Lets read point file:" << endl;
		vector<vector<Scan_Point*>> mat_p_scp = TXTReader::Read_point_PTX_to_matrix_of_pointers(folder_name + file_name + ".ptx");
		cout << mat_p_scp.size() << " x " << mat_p_scp[0].size() << " = " << mat_p_scp.size() * mat_p_scp[0].size() << " points read." << endl;

		cout << "Rectangular grayscale notstretched image creation:" << endl;
		Mat image_rect_gray_nstr_05 = Matrix_of_Scanpoints_to_Rectangular_Image(mat_p_scp, 1);
		imwrite(folder_name + file_name + "_rect_gray_nstr_05.png", image_rect_gray_nstr_05);
		Mat image_rect_gray_nstr_05_hist = createAndSaveHistogram(image_rect_gray_nstr_05);
		imwrite(folder_name + file_name + "_hist_nstr_05.png", image_rect_gray_nstr_05_hist);
		cout << "Rectangular grayscale notstretched image and its histogram are saved." << endl;

		cout << "Blacken weird pixels" << endl;
		assignBlackColorToWeirdPixels(mat_p_scp); // assign intensity 0 to all pixels with (x,y,z) coordinates equal to (0,0,0) 

		cout << "Rectangular grayscale notstretched image after blackening weird pixels creation:" << endl;
		Mat image_rect_gray_nstr = Matrix_of_Scanpoints_to_Rectangular_Image(mat_p_scp, 1);
		imwrite(folder_name + file_name + "_rect_gray_nstr.png", image_rect_gray_nstr);
		Mat image_rect_gray_nstr_hist = createAndSaveHistogram(image_rect_gray_nstr);
		imwrite(folder_name + file_name + "_hist_nstr.png", image_rect_gray_nstr_hist);
		cout << "Rectangular grayscale notstretched image after blackening weird pixels and its histogram are saved." << endl;

		cout << "Stretch intensity:" << endl;
		Stretch_intensity_2D(mat_p_scp);

		// create and save rectangular grayscale image	
		cout << "Rectangular grayscale stretched image creation:" << endl;
		Mat image_rect_gray_str = Matrix_of_Scanpoints_to_Rectangular_Image(mat_p_scp, 1);
		imwrite(folder_name + file_name + "_rect_gray_str.png", image_rect_gray_str);
		Mat image_rect_gray_str_hist = createAndSaveHistogram(image_rect_gray_str);
		imwrite(folder_name + file_name + "_hist_str.png", image_rect_gray_str_hist);
		cout << "Rectangular grayscale stretched image and its histogram are saved." << endl;

		// create and save rectangular color image
		cout << "Rectangular color image creation:" << endl;
		Mat image_rect_RGB = Matrix_of_Scanpoints_to_Rectangular_Image(mat_p_scp, 0);
		imwrite(folder_name + file_name + "_rect_RGB.png", image_rect_RGB);
		cout << "Rectangular color image saved." << endl;

		// transform 2D to 1D, delete zeros and stretch intensity
		cout << "Transform matrix to vector and delete zeros:" << endl;
		vector<Scan_Point*> p_points = Matrix_2D_to_1D(mat_p_scp);
		Delete_zeros_1D(p_points); // Projective image cannot be built if weird points are present in the cloud
		cout << p_points.size() << " points after zeros deleting. " << endl;

		// create and save projected grayscale image
		cout << "Non-distorted grayscale image creation:" << endl;
		Mat image_proj_gray = Vector_of_Scanpoints_to_Projected_Image(p_points, 1, 1.2 * max(mat_p_scp[0].size(), mat_p_scp.size()));
		imwrite(folder_name + file_name + "_proj_gray.png", image_proj_gray);
		cout << "Non-distorted grayscale image saved." << endl;

		// create and save projected color image
		cout << "Non-distorted color image creation:" << endl;
		Mat image_proj_RGB = Vector_of_Scanpoints_to_Projected_Image(p_points, 0, 1.2 * max(mat_p_scp[0].size(), mat_p_scp.size()));
		imwrite(folder_name + file_name + "_proj_RGB.png", image_proj_RGB);
		cout << "Non-distorted color image saved." << endl;

		// detect keypoints from image using certain parameters and save them with image
		cout << "Keypoints detection:" << endl;
		vector<KeyPoint> keypoints_rect_gray = detectAndSaveInterestPoints(image_rect_gray_str, kSize, thresh_wei, thresh_iso, 1, folder_name + file_name + "_image_with_kpts_gray");
		convertKeypointsToScanPoints(keypoints_rect_gray, mat_p_scp, folder_name + file_name + "_kpts_gray.ptx");
		cout << "Processing of image is finished. Number of detected keypoints is " << keypoints_rect_gray.size() << endl;
	}
	else if (users_choise == 1 || users_choise == 2) {
		// Processing point clouds of TU interior
		thresh_iso = 0.75; //0.75 for interior, 0.99 for exterior dataset
		if (users_choise == 1) file_name = "a87aaa504";
		else if (users_choise == 2) file_name = "a87aaa505";

		// read initial pointcloud
		cout << "Lets read point file:" << endl;
		vector<vector<Scan_Point*>> mat_p_scp = TXTReader::Read_point_PTX_to_matrix_of_pointers(folder_name + file_name + ".ptx");
		cout << mat_p_scp.size() << " x " << mat_p_scp[0].size() << " = " << mat_p_scp.size() * mat_p_scp[0].size() << " points read." << endl;

		assignBlackColorToWeirdPixels(mat_p_scp);
		Stretch_intensity_2D(mat_p_scp);

		// create and save rectangular grayscale image	
		cout << "Rectangular grayscale stretched image creation:" << endl;
		Mat image_rect_gray_str = Matrix_of_Scanpoints_to_Rectangular_Image(mat_p_scp, 1);
		imwrite(folder_name + file_name + "_rect_gray_str.png", image_rect_gray_str);
		cout << "Rectangular grayscale image saved." << endl;

		// detect keypoints from image using certain parameters and save them with image
		cout << "Keypoints detection:" << endl;
		vector<KeyPoint> keypoints_rect_gray = detectAndSaveInterestPoints(image_rect_gray_str, kSize, thresh_wei, thresh_iso, 1, folder_name + file_name + "_image_with_kpts_gray");
		convertKeypointsToScanPoints(keypoints_rect_gray, mat_p_scp, folder_name + file_name + "_kpts_gray.ptx");
		cout << "Processing of image is finished. Number of pts is " << keypoints_rect_gray.size() << endl;
	}
	else cout << "Wrong choise." << endl;

	system("PAUSE");
	return 0;
}