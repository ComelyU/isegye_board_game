from typing import Annotated
from fastapi import FastAPI, File, UploadFile, Form
from fastapi.responses import FileResponse, Response
from pydantic import BaseModel
from swap_face import swap
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
    
    sourceFile = base64.b64decode(files.sourceFile)
    themeFile = base64.b64decode(files.themeFile)   
    
    newFile = swap(sourceFile, themeFile)

    return newFile
