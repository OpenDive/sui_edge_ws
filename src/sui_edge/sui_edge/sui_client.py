from typing import Optional, List, Dict, Any
import asyncio
import logging
from pysui.sui.sui_config import SuiConfig
from pysui.sui.sui_clients.sync_client import SuiClient
from pysui.sui.sui_types.address import SuiAddress
from pysui.sui.sui_builders.get_builders import GetTransaction
from pysui.sui.sui_builders.subscription_builders import SubscribeTransaction

class SuiBlockchainClient:
    """Pure blockchain client for Sui network."""
    
    def __init__(self, network: str, rpc_url: Optional[str] = None, ws_url: Optional[str] = None, keypair_path: Optional[str] = None):
        self.logger = logging.getLogger("sui_blockchain_client")
        self.network = network
        
        # Initialize Sui configuration
        self.config = SuiConfig.user_config(
            rpc_url=rpc_url or f"https://fullnode.{network}.sui.io:443",
            ws_url=ws_url or f"wss://fullnode.{network}.sui.io:443",
            # If keypair_path is provided, we'll need to load the keys from it
            prv_keys=[]  # We'll load these from keypair_path if provided
        )
        
        # Initialize Sui client
        self.client: Optional[SuiClient] = None
        
    async def connect(self) -> None:
        """Initialize connection to Sui network."""
        try:
            self.client = SuiClient(self.config)
            
            # Test connection by getting latest checkpoint
            result = self.client.get_latest_checkpoint_sequence_number()
            if not result.is_ok():
                raise Exception(f"Failed to connect: {result.result_string}")
                
            self.logger.info(f'Connected to Sui {self.network}')
            
        except Exception as e:
            self.logger.error(f'Failed to connect to Sui network: {str(e)}')
            raise

    async def get_status(self) -> Dict[str, Any]:
        """Get current network status."""
        if not self.client:
            raise RuntimeError("Client not initialized")
        
        try:
            result = self.client.get_latest_checkpoint_sequence_number()
            if not result.is_ok():
                raise Exception(f"Failed to get status: {result.result_string}")
                
            return {
                "network": self.network,
                "latest_checkpoint": result.result_data,
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
            # Create transaction builder
            builder = GetTransaction.submit_tx(
                tx_bytes=tx_bytes,
                signatures=[signature],
                request_type="WaitForLocalExecution"
            )
            
            # Execute transaction
            result = self.client.execute(builder)
            if not result.is_ok():
                return {
                    "success": False,
                    "status": "error",
                    "error": result.result_string
                }
            
            tx_response = result.result_data
            return {
                "success": True,
                "digest": tx_response.digest,
                "status": "success",
                "effects": tx_response.effects
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
            # Query events using the event subscription builder
            builder = SubscribeTransaction.subscribe_event(
                cursor=cursor,
                limit=limit
            )
            result = self.client.execute(builder)
            
            if not result.is_ok():
                return {
                    "success": False,
                    "error": result.result_string
                }
            
            events_data = result.result_data
            return {
                "success": True,
                "events": events_data.data,
                "next_cursor": events_data.next_cursor
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
            result = self.client.get_object(object_id)
            if not result.is_ok():
                return {
                    "success": False,
                    "error": result.result_string
                }
            
            return {
                "success": True,
                "object_info": result.result_data
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
            # The sync client doesn't have an explicit close method
            self.client = None
