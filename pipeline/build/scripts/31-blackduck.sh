#!/bin/bash

apt-get update        
apt-get -y --no-install-recommends install default-jdk

# Set environment variables
export SCAN_CLI_OPTS=" -Duser.language=en -Duser.country=US"
export DETECT_JAR_DOWNLOAD_DIR=$HOME

# Get Black Duck Bearer Token
BLACKDUCK_BEARER_TOKEN=$(curl -s -X POST -H 'Accept: application/vnd.blackducksoftware.user-4+json' -H "Authorization: token ${BLACKDUCK_TOKEN}" "${BLACKDUCK_PLATFORM}/api/tokens/authenticate" | jq -r .bearerToken)
echo $BLACKDUCK_BEARER_TOKEN

# Get Project Version URL
PROJECT_VERSION_URL=$(curl -s -X GET -H 'Accept: application/vnd.blackducksoftware.project-detail-4+json' -H "Authorization: Bearer $BLACKDUCK_BEARER_TOKEN" "${BLACKDUCK_PLATFORM}/api/projects?limit=1000" | jq --arg repository "${BLACKDUCK_PREFIX}${BLACKDUCK_PROJECT}" '.items[] | select(.name | contains($repository)) | ._meta.links[0].href' -r)
echo $PROJECT_VERSION_URL

# Get Scan Count
BLACKDUCK_SCAN_COUNT=$(curl -s -X GET -H 'Accept: application/vnd.blackducksoftware.project-detail-4+json' -H "Authorization: Bearer $BLACKDUCK_BEARER_TOKEN" $PROJECT_VERSION_URL | jq '.items[].versionName' | wc -l)
echo $BLACKDUCK_SCAN_COUNT

# Delete oldest scan if there are more than 10 scans
if test $BLACKDUCK_SCAN_COUNT -ge 10; then
    echo "Existing more than 10 Versions"
    BLACKDUCK_OLDEST_SCAN=$(curl -s -X GET -H "Accept: application/vnd.blackducksoftware.project-detail-4+json" -H "Authorization: Bearer ${BLACKDUCK_BEARER_TOKEN}" ${PROJECT_VERSION_URL} | jq "[.items[] | {name: .versionName, created: .createdAt, reportLink: ._meta.href }]" -r -c | jq -c -s '.[] | sort_by(.created)'  | jq -r '.[0].reportLink')
    
    if [ -n "$BLACKDUCK_OLDEST_SCAN" ]; then
        curl -s -X DELETE -H "Accept: application/vnd.blackducksoftware.project-detail-4+json" -H "Authorization: Bearer ${BLACKDUCK_BEARER_TOKEN}" "${BLACKDUCK_OLDEST_SCAN}"
        echo "Deleting the oldest version"
    else
        echo "No scan found to delete."
    fi
fi

# Run Synopsys Detect with specified options
curl -s https://detect.synopsys.com/detect9.sh | bash -s - \
    --blackduck.url="${BLACKDUCK_PLATFORM}" \
    --blackduck.api.token="${BLACKDUCK_TOKEN}" \
    --detect.project.name="${BLACKDUCK_PREFIX}${BLACKDUCK_PROJECT}" \
    --detect.project.version.name="${BLACKDUCK_NAME}_$(date --utc +%Y-%m-%d_%H:%M:%S)" \
    --detect.project.version.phase="${BLACKDUCK_PHASE}" \
    --detect.policy.check.fail.on.severities="BLOCKER,CRITICAL,MAJOR" \
    --blackduck.trust.cert=true \
    --blackduck.hub.auto.import.cert=true \
    --detect.timeout=3600 \
    --detect.source.path="${BLACKDUCK_SOURCE}" \
    --detect.included.detector.types=ALL \
    --detect.excluded.directories="${BLACKDUCK_EXCLUDE}" \
    --detect.detector.search.depth=5 \
    --detect.blackduck.signature.scanner.snippet.matching="SNIPPET_MATCHING" \
    --detect.blackduck.signature.scanner.license.search=true \
    --detect.blackduck.signature.scanner.upload.source.mode=true \
    --detect.blackduck.signature.scanner.copyright.search=true \
    --detect.cleanup=true \
    --detect.blackduck.scan.mode="INTELLIGENT" \
    --detect.conan.include.build.dependencies=true \
    --detect.accuracy.required=NONE \
    --detect.wait.for.results=false \
    --detect.excluded.detector.types="PIP,SETUPTOOLS"

# Capture the exit code
SCAN_EXIT_CODE=$?

# Check the exit code and handle accordingly
if [ $SCAN_EXIT_CODE -eq 0 ]; then
    echo "Black Duck scan completed successfully."
elif [ $SCAN_EXIT_CODE -eq 3 ]; then
    echo "Warning: Black Duck scan exited with code 3."
    # Continue with script execution or handle as needed
elif [ $SCAN_EXIT_CODE -eq 2 ]; then
    echo "Warning: FAILURE_TIMEOUT Black Duck scan exited with code 2 ."
    # Continue with script execution or handle as needed
elif [ $SCAN_EXIT_CODE -eq 1 ]; then
    echo "Warning: FAILURE_BLACKDUCK_CONNECTIVITY Black Duck scan exited with code 1 ."
    # Continue with script execution or handle as needed
else
    echo "Error: Black Duck scan failed with exit code $SCAN_EXIT_CODE."
    # Handle the failure, possibly exiting the script with a non-zero code
    exit 0
fi

#Cleanup Blackduck sources
rm /root/synopsys-detect*
rm -rf /root/blackduck
echo "Blackduck CleanUp done."