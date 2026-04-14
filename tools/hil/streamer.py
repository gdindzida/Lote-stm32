from abc import ABC, abstractmethod
from typing import Optional, Tuple, Any

import numpy as np


class DatasetStreamerAdapter(ABC):
    @abstractmethod
    def process(
        self,
        input: Optional[
            Tuple[np.ndarray[Any, Any] | None, np.ndarray[Any, Any] | None]
        ],
    ):
        """Processes next resource from dataset streamer."""
        pass


class DatasetStreamer(ABC):
    @abstractmethod
    def next(
        self,
    ) -> Optional[Tuple[np.ndarray[Any, Any] | None, np.ndarray[Any, Any] | None]]:
        """Return the next resource. Returns None when dataset is exhausted."""
        pass

    @abstractmethod
    def has_next(self) -> bool:
        """Returns True if it has next resource."""
        pass

    @abstractmethod
    def reset(self) -> None:
        """Restart streaming from the beginning."""
        pass

    @abstractmethod
    def run(self) -> None:
        """Runs stream."""
        pass
