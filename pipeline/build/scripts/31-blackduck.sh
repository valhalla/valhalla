#!/bin/bash

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

blackduck-c-cpp \
    -d "${BLACKDUCK_SOURCE}" \
    -bc "cd ${BLACKDUCK_SOURCE} && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_COMPILER=gcc -DENABLE_SINGLE_FILES_WERROR=Off -DENABLE_THREAD_SAFE_TILE_REF_COUNT=On && make all" \
    -proj "${BLACKDUCK_PREFIX}${BLACKDUCK_PROJECT}" \
    -vers "${BLACKDUCK_NAME}_$(date --utc +%Y-%m-%d_%H:%M:%S)" \
    -bd "${BLACKDUCK_PLATFORM}" \
    -a "${BLACKDUCK_TOKEN}" \
    -sh

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
rm -rf /root/.synopsys
echo "Blackduck CleanUp done."