#!/usr/bin/env python3
"""
Google Images URL collector for car interior mess detection
Provides direct Google Images links for manual download
"""

import webbrowser
import urllib.parse

def generate_google_image_urls():
    """
    Generate Google Images URLs for car interior mess searches
    """
    
    # Specific search queries for car interior messes
    search_queries = [
        # Bottom of car seat searches
        "dirty car seat bottom crumbs",
        "under car seat mess debris", 
        "car seat crevice dirt crumbs",
        "car floor mat crumbs trash",
        "between car seats garbage mess",
        
        # Specific mess types
        "car interior food crumbs close up",
        "french fries under car seat",
        "chip crumbs car interior",
        "cookie crumbs car seat",
        "spilled snacks car floor",
        
        # Real detailing before/after
        "car detailing before vacuum seat",
        "dirty car interior professional cleaning",
        "car seat deep cleaning before after",
        "automotive interior detailing mess",
        
        # Specific problem areas
        "car door pocket trash mess",
        "center console cup holder sticky mess",
        "car seat belt buckle area crumbs",
        "driver seat side pocket debris"
    ]
    
    print("🔍 Google Images Direct Links for Car Interior Mess")
    print("=" * 60)
    print("\nClick each link to open in browser and manually save images:")
    print("(Right-click on images → Save Image As...)\n")
    
    urls = []
    for i, query in enumerate(search_queries, 1):
        encoded_query = urllib.parse.quote(query)
        url = f"https://www.google.com/search?q={encoded_query}&tbm=isch&tbs=isz:l"
        urls.append(url)
        print(f"{i}. {query}")
        print(f"   {url}\n")
    
    print("\n💡 Tips for manual collection:")
    print("1. Look for HIGH RESOLUTION images showing actual mess")
    print("2. Focus on CLOSE-UP shots of crumbs, wrappers, debris")
    print("3. Save images showing DIFFERENT ANGLES of car seats")
    print("4. Include both LIGHT and DARK car interiors")
    print("5. Get images with VARYING LIGHTING conditions")
    
    print("\n📁 Suggested organization:")
    print("Create folders:")
    print("  - images/crumbs/")
    print("  - images/wrappers/")
    print("  - images/stains/")
    print("  - images/mixed_mess/")
    print("  - images/clean_reference/")
    
    return urls

def open_urls_in_browser(urls, batch_size=5):
    """
    Optionally open URLs in browser in batches
    """
    print("\n🌐 Open in browser? (y/n): ", end="")
    response = input().strip().lower()
    
    if response == 'y':
        for i in range(0, len(urls), batch_size):
            batch = urls[i:i+batch_size]
            print(f"\nOpening batch {i//batch_size + 1}...")
            for url in batch:
                webbrowser.open(url)
            
            if i + batch_size < len(urls):
                print("Press Enter to open next batch...")
                input()

def main():
    urls = generate_google_image_urls()
    
    print("\n" + "=" * 60)
    print("📸 Ready to collect images!")
    print("\nWould you like to automatically open these URLs in your browser?")
    open_urls_in_browser(urls)
    
    print("\n✅ Next steps:")
    print("1. Create an 'images' folder in this directory")
    print("2. Manually save the most relevant images from Google")
    print("3. Organize them by type (crumbs, wrappers, etc.)")
    print("4. Aim for 20-50 high-quality images to start")

if __name__ == "__main__":
    main()