from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
# Import skills from the skills module
from skills import validate_gpu, search_knowledge_base, generate_node, to_urdu

app = FastAPI()

# Allow EVERYONE (Hackathon Mode)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Wildcard: Works on Localhost & Vercel
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class TranslationRequest(BaseModel):
    text: str

class HardwareCheckRequest(BaseModel):
    gpu_model: str
    ram_gb: int

class KnowledgeBaseRequest(BaseModel):
    query: str

class ROSNodeRequest(BaseModel):
    name: str
    topic: str
    node_type: str

@app.post("/api/translate")
async def translate_text(request: TranslationRequest):
    # Use the translate skill
    result = to_urdu(request.text)
    return {"translated_text": result}

@app.post("/api/hardware-check")
async def hardware_check(request: HardwareCheckRequest):
    # Use the hardware check skill
    result = validate_gpu(request.gpu_model, request.ram_gb)
    return {"result": result}

@app.post("/api/knowledge-base")
async def knowledge_base_search(request: KnowledgeBaseRequest):
    # Use the RAG retrieval skill
    result = search_knowledge_base(request.query)
    return {"response": result}

@app.post("/api/ros-node")
async def generate_ros_node(request: ROSNodeRequest):
    # Use the ROS2 code generation skill
    result = generate_node(request.name, request.topic, request.node_type)
    return {"node_code": result}

@app.get("/")
def health_check():
    return {"status": "Vector Agent is Online"}