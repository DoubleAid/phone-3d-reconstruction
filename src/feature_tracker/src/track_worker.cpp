#include "track_worker.hpp"

// 判断一个光流点
int inBorder(const cv::Point2f &pt) {
    // const int BORDER_SIZE = 1
}

TrackWorker::TrackWorker() {
}

void TrackWorker::readImage(const cv::Mat &_img, double _cur_time) {
    cv::Mat img;
    if (equalize_) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(_img, img);
        Logger::info("clahe equalize finished");
    } else {
        img = _img;
    }

    if (forw_img_.empty()) {
        prev_img_ = cur_img_ = forw_img_ = img;
    }
    else
    {
        forw_img_ = img;
    }

    forw_pts_.clear();

    if (cur_pts_.size() > 0) {
        vector<uchar> status;       // 跟踪成功1， 失败0
        vector<float> err;          // 跟踪误差

        cv::calcOpticalFlowPyrLK(cur_img_, forw_img_, cur_pts_, forw_pts_, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts_.size()); i++) {
            // if (status[i] && !inBorder())
        }
    }
}

bool TrackWorker::updateID(unsigned int i) {

}

git remote add origin git@github.com:DoubleAid/phone-3d-reconstruction.git
git branch -M main
git push -u origin main