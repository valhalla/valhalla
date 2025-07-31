#!/bin/bash

echo "<<< send Microsoft Teams notification >>>"
curl "${TEAMS_URL}" \
-H "Content-Type:application/json" \
--data @<(cat <<EOF
  {
     "type":"message",
     "attachments":[
        {
           "contentType":"application/vnd.microsoft.card.adaptive",
           "contentUrl":null,
           "content":{
              "type": "AdaptiveCard",
              "$schema": "http://adaptivecards.io/schemas/adaptive-card.json",
              "version": "1.4",
              "body": [
                  {
                      "type": "TextBlock",
                      "text": "$APP_NAME",
                      "wrap": true,
                      "weight": "Bolder",
                      "size": "Large"
                  },
                  {
                      "type": "TextBlock",
                      "text": "Failed to build branch '$BUILD_SOURCEBRANCHNAME'",
                      "wrap": true
                  }
              ]
          }
        }
     ]
  }
EOF
)