#pragma once

enum class GFMMState {ERROR=-1, NOT_READY=0, READY_TO_START=2, HEADING_TOWARD_ENTRANCE=3, SEARCHING_FOR_FIRE=4, 
                                        APPROACHING_FIRE=5, EXTINGUISHING_FIRE=6, RETURNING_TO_BASE=7, FINISHED=8};