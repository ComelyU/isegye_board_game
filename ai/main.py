from typing import Annotated
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from swap_face import swap
from recommendation import get_recommendations, get_list
import base64


class CreateImageRequest(BaseModel):
    sourceFile: str
    themeFile: str


app = FastAPI()


@app.get("/health")
def check_health():
    return {"status": "ok"}


@app.post("/ai/swap")
async def create_swap_image(files: CreateImageRequest):
    if not files.sourceFile:
        return {"message": "No file sent"}
    
    source_file = base64.b64decode(files.sourceFile)
    theme_file = base64.b64decode(files.themeFile)
    
    new_file = swap(source_file, theme_file)

    return new_file


@app.post("/ai/recommendation")
async def get_recommendation(game_id: int):
    if game_id:
        if game_id not in get_list():
            raise HTTPException(
                status_code=404,
                detail="Game not found"
            )
        return get_recommendations(int(game_id))

    return
