import requests
import json
import os
import time
from datetime import datetime

# --- Configuration ---
API_URL = "https://mosaic-plaza-aanbodapi.zig365.nl/api/v1/actueel-aanbod?limit=60&locale=nl_NL&page=0&sort=!reactionData.zoekprofielMatchOrder,-reactionData.zoekprofielMatchOrder,%2BreactionData.aangepasteTotaleHuurprijs"
SEEN_FILE = "seen_lampendriessen.json"
REQUEST_TIMEOUT = 30  # seconds, so a hung server can never freeze the bot

# --- Telegram Settings ---
TELEGRAM_TOKEN = "YOUR_BOT_TOKEN_HERE"  # Paste your API token here
TELEGRAM_CHAT_ID = "YOUR_CHAT_ID_HERE"  # Paste your Chat ID here

def send_telegram_message(message):
    """Sends a text message to your phone via Telegram. Returns True on success."""
    url = f"https://api.telegram.org/bot{TELEGRAM_TOKEN}/sendMessage"
    payload = {
        "chat_id": TELEGRAM_CHAT_ID,
        "text": message,
        "parse_mode": "HTML"
    }
    try:
        response = requests.post(url, data=payload, timeout=REQUEST_TIMEOUT)
        if response.ok:
            return True
        print(f"⚠️ Telegram rejected the message. HTTP Status: {response.status_code} | {response.text}")
        return False
    except Exception as e:
        print(f"Failed to send Telegram message: {e}")
        return False

def load_seen_houses():
    """Loads the list of house IDs we have already seen."""
    if os.path.exists(SEEN_FILE):
        try:
            with open(SEEN_FILE, 'r') as file:
                return set(json.load(file))
        except (json.JSONDecodeError, ValueError) as e:
            print(f"⚠️ Could not read {SEEN_FILE} ({e}). Starting with an empty list.")
    return set()

def save_seen_houses(seen_houses):
    """Saves the updated list of house IDs to disk atomically."""
    tmp_file = f"{SEEN_FILE}.tmp"
    with open(tmp_file, 'w') as file:
        json.dump(list(seen_houses), file)
    os.replace(tmp_file, SEEN_FILE)  # atomic on the same filesystem; never leaves a half-written file

def fetch_houses():
    """Fetches the latest JSON data from the API."""
    headers = {
        "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36",
        "Accept": "application/json",
        "Referer": "https://plaza.newnewnew.space/"
    }
    try:
        response = requests.get(API_URL, headers=headers, timeout=REQUEST_TIMEOUT)

        # If the server is happy, return the data
        if response.status_code == 200:
            return response.json()

        # If the server blocks us, print the exact error code
        else:
            print(f"⚠️ Server rejected the request. HTTP Status: {response.status_code}")
            return None

    except Exception as e:
        print(f"⚠️ Network error: {e}")
        return None

def main():
    data = fetch_houses()

    # --- THE BULLETPROOF GUARD ---
    # If data is None or not a dictionary, safely abort this cycle without crashing
    if not data or not isinstance(data, dict):
        print("Skipping this cycle due to missing or invalid data.\n")
        return
    # -----------------------------

    properties = data.get("data", data.get("result", data.get("items", data)))

    if not isinstance(properties, list):
        print("Data received, but couldn't find the property list.\n")
        return

    seen_houses = load_seen_houses()
    new_listings = []

    for prop in properties:
        if not isinstance(prop, dict):
            continue

        adres = prop.get("adres") or {}  # tolerate "adres": null, not just a missing key
        prop_id = str(prop.get("id") or prop.get("objectId") or prop.get("objectnummer", "UNKNOWN"))

        street = str(
            prop.get("straat") or
            prop.get("street") or
            adres.get("straat", "") or
            prop.get("title", "")
        ).lower()

        if "lampendriessen" in street and prop_id not in seen_houses:
            new_listings.append(prop)
            seen_houses.add(prop_id)

    if new_listings:
        print(f"🎉 Found {len(new_listings)} NEW listing(s)! Sending notification...\n")
        telegram_text = f"🚨 <b>{len(new_listings)} New Listing(s) at Luna!</b>\n\n"

        for house in new_listings:
            adres = house.get("adres") or {}
            reaction_data = house.get("reactionData") or {}

            street_display = house.get("straat") or adres.get("straat") or "De Lampendriessen"
            huisnummer = house.get("huisnummer") or adres.get("huisnummer", "")
            toevoeging = house.get("huisnummerToevoeging") or adres.get("toevoeging", "")

            full_address = f"{street_display} {huisnummer} {toevoeging}".strip()
            price = house.get("totaleHuurprijs") or house.get("huurprijs") or reaction_data.get("aangepasteTotaleHuurprijs", "Unknown")

            print(f"- {full_address} | €{price}")
            telegram_text += f"🏠 {full_address}\n💶 €{price}\n\n"

        telegram_text += "<a href='https://plaza.newnewnew.space/aanbod/wonen'>Go to Plaza to apply!</a>"

        # Only mark these houses as "seen" if the alert actually went out — otherwise a
        # failed send would silently swallow the listing and you'd never be notified again.
        if send_telegram_message(telegram_text):
            save_seen_houses(seen_houses)
        else:
            print("⚠️ Notification failed to send; will retry these listings next cycle.\n")
    else:
        print("No new listings found.\n")

if __name__ == "__main__":
    print("Bot started! Checking De Lampendriessen every 2 minutes.")
    print("Press Ctrl+C in this terminal to stop.\n")

    while True:
        current_time = datetime.now().strftime("%H:%M:%S")
        print(f"[{current_time}] Pinging server...")

        # Guard the whole cycle so one unexpected error can never kill the loop.
        try:
            main()
        except Exception as e:
            print(f"⚠️ Unexpected error this cycle: {e}\n")

        time.sleep(120)
