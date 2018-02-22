/* Copyright (c) 2018 Arrow Electronics, Inc.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Apache License 2.0
 * which accompanies this distribution, and is available at
 * http://apache.org/licenses/LICENSE-2.0
 * Contributors: Arrow Electronics, Inc.
 */

#include "arrow/state.h"
#include <arrow/utf8.h>
#include <debug.h>
#include <sys/type.h>
#include "ArrowConnect.h"

int state_handler(char *str)
{
    int Status = 0;
    char json_str_buffer[64] = {0};
    DBG("State Handler Payload: %s",str);

    JsonNode *main_ = json_decode(str);
    JsonNode *deviceStateNode = NULL;
    JsonNode *grandchild = NULL;

    if(main_)
    {
        json_foreach(deviceStateNode, main_) {
            if(strcmp(deviceStateNode->key, "rgbValues") == 0)
            {
                int Red = -1;
                int Green = -1;
                int Blue = -1;
                JsonNode *OriginalValue = NULL;
                OriginalValue = json_find_member(deviceStateNode, "value");

                JsonNode *ValueObject = json_mkobject();
                if(OriginalValue)
                {
                    sprintf(json_str_buffer, "{\"value\":%s}",OriginalValue->string_);
                    DBG("Original value: %s",OriginalValue->string_);
                    DBG("Re-formed value: %s", json_str_buffer);
                    ValueObject = json_decode(json_str_buffer);
                    if(ValueObject)
                    {
                        JsonNode *ArrayKey = json_find_member(ValueObject, "value");
                        JsonNode *ArrayElement = NULL;
                        int ListIndex = 0;
                        json_foreach(ArrayElement, ArrayKey){
                            switch(ListIndex)
                            {
                            case 0:
                                Red = ArrayElement->number_;
                                break;
                            case 1:
                                Green = ArrayElement->number_;
                                break;
                            case 2:
                                Blue = ArrayElement->number_;
                                break;
                            default:
                                DBG("Should not get here!!!");
                                break;
                            }

                            ListIndex++;
                        }

                        DBG("Got colors: %d %d %d", Red, Green, Blue);
                        color C = {Red, Green, Blue};
                        show_color(C);

                        //add_state("rgbValues","[123,255,45]");
                        //arrow_post_state_update(current_device());
                    }
                    else
                    {
                        DBG("Failed to parse new value object");
                    }
                }
                else
                {
                    DBG("Failed to find member: \"value\"");
                }
            }
            else
            {
                DBG("Found node: %s", deviceStateNode->key);
            }
        }
    }

#if 0
    if(main_)
    {
        DBG("main_->tag: %d",main_->tag);
        int Red = -1;
        int Green = -1;
        int Blue = -1;
        JsonNode *rgbList = NULL;
        JsonNode *rgbNode = json_find_member(main_, "rgbValues");
        if(rgbNode && rgbNode->tag == JSON_OBJECT)
        {
            rgbList = json_find_member(rgbNode, "value");
        }
        else
        {
            DBG("Could not find member: rgbValues");
            if(rgbNode)
            {
                DBG("rgbNode->tag: %d",rgbNode->tag);
            }
            Status = -1;
        }

        if(rgbList && rgbList->tag == JSON_ARRAY)
        {

            int ListIndex = 0;
            for(JsonNode *CurrentListNode = rgbList->children.head; CurrentListNode != NULL; CurrentListNode = CurrentListNode->next)
            {
                switch(ListIndex)
                {
                case 0:
                    Red = CurrentListNode->number_;
                    break;
                case 1:
                    Green = CurrentListNode->number_;
                    break;
                case 2:
                    Blue = CurrentListNode->number_;
                    break;
                default:
                    DBG("Should not get here!!!");
                    break;
                }

                ListIndex++;
            }
        }
        else
        {
            if(rgbList)
            {
                DBG("rgbList->tag: %d",rgbList->tag);
            }
            else
            {
                DBG("Could not find member: value");
            }

            Status = -1;
        }

        json_delete(main_);

        DBG("Got colors: %d %d %d", Red, Green, Blue);

        //show_color(RGBColor);
    }
    else
    {
        DBG("json parse failed");
        Status = -1;
    }
#endif

  return Status;
}
