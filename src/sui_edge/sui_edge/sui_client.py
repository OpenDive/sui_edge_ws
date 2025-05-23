from typing import Optional, List, Dict, Any
import asyncio
from sui.client import SuiClient
from sui.types import SuiTransactionResponse, SuiEvent
import logging

class SuiBlockchainClient:
    """Pure blockchain client for Sui network interactions."""
    
    def __init__(self, network: str, rpc_url: Optional[str] = None, ws_url: Optional[str] = None):
        self.logger = logging.getLogger("sui_blockchain_client")
        self.network = network
        self.rpc_url = rpc_url
        self.ws_url = ws_url
        self.client: Optional[SuiClient] = None
        
    async def connect(self) -> None:
        """Initialize connection to Sui network."""
        try:
            self.client = SuiClient(
                config={
                    "rpc_url": self.rpc_url,
                    "ws_url": self.ws_url,
                }
            )
            # Test connection
            await self.client.get_latest_checkpoint_sequence_number()
            self.logger.info(f'Connected to Sui {self.network}')
        except Exception as e:
            self.logger.error(f'Failed to connect to Sui network: {str(e)}')
            raise

    async def get_status(self) -> Dict[str, Any]:
        """Get current network status."""
        if not self.client:
            raise RuntimeError("Client not initialized")
        
        try:
            checkpoint = await self.client.get_latest_checkpoint_sequence_number()
            return {
                "network": self.network,
                "latest_checkpoint": checkpoint,
                "connected": True
            }
        except Exception as e:
            self.logger.error(f'Failed to get status: {str(e)}')
            raise

    async def submit_transaction(self, tx_bytes: str, signature: str) -> Dict[str, Any]:
        """Submit a transaction to the Sui network."""
        if not self.client:
            raise RuntimeError("Client not initialized")
        
        try:
            response = await self.client.execute_transaction(
                tx_bytes=tx_bytes,
                signature=signature,
                request_type="WaitForLocalExecution"
            )
            return {
                "success": True,
                "digest": response.digest,
                "status": "success",
                "effects": response.effects
            }
        except Exception as e:
            self.logger.error(f'Transaction submission failed: {str(e)}')
            return {
                "success": False,
                "status": "error",
                "error": str(e)
            }

    async def get_events(self, cursor: Optional[str] = None, limit: int = 50) -> Dict[str, Any]:
        """Query events from the Sui network."""
        if not self.client:
            raise RuntimeError("Client not initialized")
        
        try:
            events = await self.client.query_events(
                query={},
                cursor=cursor,
                limit=limit
            )
            return {
                "success": True,
                "events": events,
                "next_cursor": events.next_cursor if hasattr(events, 'next_cursor') else None
            }
        except Exception as e:
            self.logger.error(f'Failed to fetch events: {str(e)}')
            return {
                "success": False,
                "error": str(e)
            }

    async def get_object(self, object_id: str) -> Dict[str, Any]:
        """Get object information from Sui."""
        if not self.client:
            raise RuntimeError("Client not initialized")
        
        try:
            object_info = await self.client.get_object(object_id)
            return {
                "success": True,
                "object_info": object_info
            }
        except Exception as e:
            self.logger.error(f'Failed to fetch object {object_id}: {str(e)}')
            return {
                "success": False,
                "error": str(e)
            }

    async def close(self) -> None:
        """Close the Sui client connection."""
        if self.client:
            await self.client.close()
            self.client = None
