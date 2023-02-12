#!/usr/bin/env groovy

/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

def CheckCoverageCpp() {
	dir(path: "${PULL_REQUEST_TO_REPO_SLUG}/") {

		echo "Jenkins script says: Checking C++ Test Coverage ....."

		// Whole pipeline shouldn't fail if these steps fail.
		sh returnStatus: true, script: "lcov --capture --directory build/ --output-file coverage.info"
		
		sh returnStatus: true, script: '''lcov --remove coverage.info -o coverage_filtered.info \
										  '/usr/*' \
										  '*/caffe-ssd/install/*' \
										  '*/Jenkins_changes/*/build*' \
										  '*/googletest/*' \
										  '/opt/*' \
										  '/tmp/*'
										  '''
		sh returnStatus: true, script: "genhtml coverage_filtered.info --output-directory c++_coverage" //This step fails with nvcc files, but won't make the whole pipeline fail
	}

	publishHTML([allowMissing: true, alwaysLinkToLastBuild: false, keepAll: true, reportDir: '${PULL_REQUEST_TO_REPO_SLUG}/c++_coverage', reportFiles: 'index.html', reportName: 'TestCoverageCpp', reportTitles: ''])
}

def CheckCoveragePython() {
	dir(path: "${PULL_REQUEST_TO_REPO_SLUG}/build") {
		echo "Jenkins script says: Checking python Test Coverage ....."
		sh returnStatus: true, script: "python3 -m coverage html -i -d python_coverage"
		
		publishHTML([allowMissing: true, alwaysLinkToLastBuild: false, keepAll: true, reportDir: 'python_coverage', reportFiles: 'index.html', reportName: 'TestCoveragePython', reportTitles: ''])
	}
}

return this
