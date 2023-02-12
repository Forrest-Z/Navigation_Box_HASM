#!/usr/bin/env groovy

/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

def ExecuteSystemTests() {
	script {
		if(("${NODE_LABEL}" == "jetson" || "${NODE_LABEL}" == "ubuntu+gpu") && "${PULL_REQUEST_TO_REPO_SLUG}" == "nie_libraries") {
			dir(path: "nie_libraries/common/apps/docker/test_system") {

				echo "Jenkins script says: System Level Testing - development dockers ..."

				sh '''
				robot --consolewidth 128 \
					  --debugfile plainlog  `# ascii debug file is generated too` \
					  --outputdir out       `# Robot puts the log files here` \
					  --exclude docker \
					  .
				'''
			}
			dir(path: "nie_libraries/common/apps/trt/test_system") {

				echo "Jenkins script says: System Level Testing - trtexec ..."

				sh '''
				robot --consolewidth 128 \
					  --debugfile plainlog  `# ascii debug file is generated too` \
					  --outputdir out_fastcnn `# Robot puts the log files here` \
					  --exclude *grind    `# do not run tests tagged with helgrind since it gives false errors specific to cuda` \
					  --exclude fps         `# do not run frames per second tests since they are not stable` \
					  --exclude unit_test   `# do not run unit tests since they are run before` \
					  --noncritical fragile `# tests tagged with this item will run but they do not affect the test result` \
					  --variable BROKER_SERVER:10.66.201.21         `# address of the message broker` \
					  --variable BROKER_LOGIN_USER_NAME:rabbitmq    `# login user name` \
					  --variable BROKER_LOGIN_PASSWD:password       `# login user password` \
                                          --exclude broker \
					  trtexec.robot
				'''
			}
			dir(path: "nie_libraries/common/apps/trt/test_system") {

				echo "Jenkins script says: System Level Testing - Json output ..."

				sh '''
				robot --consolewidth 128 \
					  --debugfile plainlog  `# ascii debug file is generated too` \
					  --outputdir out_trtjson `# Robot puts the log files here` \
                                          --exclude docker \
					  trtjson.robot
				'''
			}
		}
	}
	script {
		if(("${NODE_LABEL}" == "jetson" || "${NODE_LABEL}" == "ubuntu+gpu") && "${PULL_REQUEST_TO_REPO_SLUG}" == "scene_segmentation") {
			dir(path: "scene_segmentation/semantic_segmentation/apps/trt/test_system") {

				echo "Jenkins script says: System Level Testing..."

				sh '''
				robot --consolewidth 128 \
					  --debugfile plainlog  `# ascii debug file is generated too` \
					  --outputdir out       `# Robot puts the log files here` \
					  --exclude helgrind    `# do not run tests tagged with helgrind since it gives false errors specific to cuda` \
					  --exclude fps         `# do not run frames per second tests since they are not stable` \
					  --exclude unit_test   `# do not run unit tests since they are run before` \
					  --noncritical fragile `# tests tagged with this item will run but they do not affect the test result` \
					  bisenet.robot
				'''
			}
			dir(path: "scene_segmentation/test_system") {

				echo "Jenkins script says: System Level Testing... - uff with trtexec"

				sh '''
				robot --consolewidth 128 \
					  --debugfile plainlog  `# ascii debug file is generated too` \
					  --outputdir out       `# Robot puts the log files here` \
					  create_engine.robot
				'''
			}
		}
	}
	script {
		if("${NODE_LABEL}" == "ubuntu" && "${PULL_REQUEST_TO_REPO_SLUG}" == "pointcloud") {
			dir(path: "pointcloud/test_sys") {

				echo "Jenkins script says: System Level Testing..."

				sh '''
				robot --consolewidth 128 \
					  --debugfile plainlog  `# ascii debug file is generated too` \
					  --outputdir out_pc    `# Robot puts the log files here` \
					  --exclude unit_test   `# do not run unit tests since they are run before` \
					  --exclude long_test   `# do not run heavy tests` \
					  --noncritical fragile `# tests tagged with this item will run but they do not affect the test result` \
					  point_cloud.robot
				'''
			}
		}
	}
	script {
		if("${NODE_LABEL}" == "ubuntu+gpu" && "${PULL_REQUEST_TO_REPO_SLUG}" == "object_detection_framework") {

			dir(path: "${PULL_REQUEST_TO_REPO_SLUG}") {

				echo "Jenkins script says: Prepare System Level Testing ..."

				if ( fileExists("${WORKSPACE}/${PULL_REQUEST_TO_REPO_SLUG}/Dockerfile") ) {
					sh '''
					cp /workspace/od/modeling/head/ThunderNet/_C* ${WORKSPACE}/${PULL_REQUEST_TO_REPO_SLUG}/od/modeling/head/ThunderNet
					cp /workspace/od/modeling/head/fcos/fcos_nms/_C* ${WORKSPACE}/${PULL_REQUEST_TO_REPO_SLUG}/od/modeling/head/fcos/fcos_nms
					'''
				} else {
					sh '''
					. /volumes1/anaconda3/bin/activate
					conda init
					conda activate od
					(cd ext && python build.py build_ext develop)
					(cd od/modeling/head/ThunderNet && python setup.py build develop)
					[ -d od/modeling/head/centernet/DCNv2 ] && (cd od/modeling/head/centernet/DCNv2 && ./make.sh) || true
					[ -d od/modeling/head/fcos ] && (cd od/modeling/head/fcos && python setup.py build develop --no-deps) || true
					'''
				}
			}
			dir(path: "${PULL_REQUEST_TO_REPO_SLUG}/test_system/infer") {

				echo "Jenkins script says: System Level Testing - infer system test ..."

				sh '''
				export HOME=/data/nie/teams/arl/system_tests_data/home/jenkins-slave
				if [ ! -e "${WORKSPACE}/${PULL_REQUEST_TO_REPO_SLUG}/Dockerfile" ]; then
					. /volumes1/anaconda3/bin/activate
					conda init
					conda activate od
				fi
				robot --consolewidth 128 \
					--debugfile plainlog `# ascii debug file is generated too` \
					--outputdir ../out_infer `# Robot puts the log files here` \
					.
				'''
			}
			dir(path: "${PULL_REQUEST_TO_REPO_SLUG}/test_system/ssd") {

				echo "Jenkins script says: System Level Testing - ssd system test ..."

				sh '''
				export HOME=/data/nie/teams/arl/system_tests_data/home/jenkins-slave
				if [ ! -e "${WORKSPACE}/${PULL_REQUEST_TO_REPO_SLUG}/Dockerfile" ]; then
					. /volumes1/anaconda3/bin/activate
					conda init
					conda activate od
				fi
				robot --consolewidth 128 \
					--debugfile plainlog `# ascii debug file is generated too` \
					--outputdir ../out_ssd `# Robot puts the log files here` \
					--variable BIN_DIR:../../build/bin `# use the locally created binaries` \
					--noncritical fragile `# tests tagged with this item will run but they do not affect the test result` \
					model_conversion.robot
				'''
			}
			dir(path: "${PULL_REQUEST_TO_REPO_SLUG}/test_system/yolov2") {

				echo "Jenkins script says: System Level Testing - yolov2 system test ..."

				sh '''
				export HOME=/data/nie/teams/arl/system_tests_data/home/jenkins-slave
				if [ ! -e "${WORKSPACE}/${PULL_REQUEST_TO_REPO_SLUG}/Dockerfile" ]; then
					. /volumes1/anaconda3/bin/activate
					conda init
					conda activate od
				fi
				robot --consolewidth 128 \
					--debugfile plainlog `# ascii debug file is generated too` \
					--outputdir ../out_yolov2 `# Robot puts the log files here` \
					--variable BIN_DIR:../../build/bin `# use the locally created binaries` \
					--noncritical fragile `# tests tagged with this item will run but they do not affect the test result` \
					model_conversion.robot
				'''
			}
			dir(path: "${PULL_REQUEST_TO_REPO_SLUG}/test_system/yolov3") {

				echo "Jenkins script says: System Level Testing - yolov3 system test ..."

				sh '''
				export HOME=/data/nie/teams/arl/system_tests_data/home/jenkins-slave
				if [ ! -e "${WORKSPACE}/${PULL_REQUEST_TO_REPO_SLUG}/Dockerfile" ]; then
					. /volumes1/anaconda3/bin/activate
					conda init
					conda activate od
				fi
				robot --consolewidth 128 \
					--debugfile plainlog `# ascii debug file is generated too` \
					--outputdir ../out_yolov3 `# Robot puts the log files here` \
					--variable BIN_DIR:../../build/bin `# use the locally created binaries` \
					--noncritical fragile `# tests tagged with this item will run but they do not affect the test result` \
					model_conversion.robot
				'''
			}
		}
	}
}

return this
