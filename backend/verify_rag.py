from main import search
import os
from dotenv import load_dotenv

load_dotenv()

def verify():
    test_query = "What is ROS 2?"
    print(f"--- Running Verification Query: '{test_query}' ---")
    
    results = search(test_query)
    
    if not results:
        print("❌ No results found. Ensure your database is populated and API keys are correct.")
        return

    print(f"✅ Found {len(results)} matches:")
    for i, res in enumerate(results):
        print(f"\n[{i+1}] Score: {res.score:.4f}")
        print(f"URL: {res.payload.get('url')}")
        # Print a snippet of the text
        text = res.payload.get('text', '')
        snippet = text[:200] + "..." if len(text) > 200 else text
        print(f"Content: {snippet}")

    # Basic heuristic check
    if any("ROS 2" in res.payload.get('text', '') for res in results):
        print("\n✨ Verification PASSED: Results contain relevant keywords.")
    else:
        print("\n⚠️ Verification WARNING: Results might not be highly relevant.")

if __name__ == "__main__":
    verify()
