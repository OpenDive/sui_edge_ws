from typing import Optional, List, Dict, Any
import asyncio
from sui.client import SuiClient
from sui.types import SuiTransactionResponse, SuiEvent
import rclpy.logging

class SuiBridgeClient:
    """Interface for interacting with the Sui blockchain."""
    
    def __init__(self, config):
        self.logger = rclpy.logging.get_logger('sui_client')
        self.config = config
        self.client: Optional[SuiClient] = None
        
    async def connect(self) -> None:
        """Initialize connection to Sui network."""
        try:
            self.client = SuiClient(
                config={
                    "rpc_url": self.config.sui_rpc_url,
                    "ws_url": self.config.sui_ws_url,
                }
            )
            # Test connection
            await self.client.get_latest_checkpoint_sequence_number()
            self.logger.info(f'Connected to Sui {self.config.sui_network}')
        except Exception as e:
            self.logger.error(f'Failed to connect to Sui network: {str(e)}')
            raise

    async def submit_transaction(self, tx_bytes: str, signature: str) -> SuiTransactionResponse:
        """Submit a transaction to the Sui network."""
        if not self.client:
            raise RuntimeError("Client not initialized")
        
        try:
            response = await self.client.execute_transaction(
                tx_bytes=tx_bytes,
                signature=signature,
                request_type="WaitForLocalExecution"
            )
            return response
        except Exception as e:
            self.logger.error(f'Transaction submission failed: {str(e)}')
            raise

    async def get_events(self, cursor: Optional[str] = None, limit: int = 50) -> List[SuiEvent]:
        """Query events from the Sui network."""
        if not self.client:
            raise RuntimeError("Client not initialized")
        
        try:
            events = await self.client.query_events(
                query={},  # Add specific query parameters as needed
                cursor=cursor,
                limit=limit
            )
            return events
        except Exception as e:
            self.logger.error(f'Failed to fetch events: {str(e)}')
            raise

    async def get_object(self, object_id: str) -> Dict[str, Any]:
        """Get object information from Sui."""
        if not self.client:
            raise RuntimeError("Client not initialized")
        
        try:
            object_info = await self.client.get_object(object_id)
            return object_info
        except Exception as e:
            self.logger.error(f'Failed to fetch object {object_id}: {str(e)}')
            raise

    def close(self) -> None:
        """Close the Sui client connection."""
        if self.client:
            asyncio.create_task(self.client.close())
            self.client = None
