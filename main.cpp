/* Copyright (c) EcarX - All Rights Reserved
 * Author: Ke Ma <ke.ma@ecarx.com.cn>
 * description: cloudpoints projection main function
 */

#include "kittiVelCamFusion.hpp"
#include <iostream>

int main(int argc, char **argv)
{

	std::string recDir = "/media/mk11/Dataset/KITTI_all/Raw/2011_09_26_drive_0009";
	std::string imgOutDir = "/home/mk11/compileWS/pclprojection/ImageOut/";

	DataVelCamFusionBase *velCamFusion_ = new KittiVelCamFusion(recDir, imgOutDir, 100);

	if (!velCamFusion_->exec())
	{
		std::cerr << "\t mk11-error: unexpected finish." << std::endl;
	}

	delete velCamFusion_;

	return 0;
}
